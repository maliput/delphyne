// https://raw.githubusercontent.com/thelink2012/any/master/test_any.hpp  -*- C++ -*-  NOLINT
//
// clalancette: tests based on upstream for the version of linb::any we are
// using.  I made some modifications below for include paths, and switching to
// use gtest in order to fit in with the rest of our testing infrastructure.
//
// Copyright (c) 2016 Denilson das Mercês Amorim
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)

// Very simplist test, could be better.

#include <gtest/gtest.h>
#include <cstdio>
#include <memory>
#include "backend/linb-any"

struct big_type {
  char i_wanna_be_big[256];
  std::string value;

  big_type() : value(std::string(300, 'b')) {
    i_wanna_be_big[0] = i_wanna_be_big[50] = 'k';
  }
};

TEST(linb_any, basics) {
  linb::any x = 4;
  linb::any y = big_type();
  linb::any z = 6;

  ASSERT_TRUE(linb::any().empty());
  ASSERT_FALSE(linb::any(1).empty());
  ASSERT_FALSE(linb::any(big_type()).empty());

  ASSERT_FALSE(x.empty());
  ASSERT_FALSE(y.empty());
  ASSERT_FALSE(z.empty());

  y.clear();
  ASSERT_TRUE(y.empty());

  x = y;
  ASSERT_TRUE(x.empty());

  z = linb::any();
  ASSERT_TRUE(z.empty());
}

TEST(linb_any, typeids) {
  ASSERT_EQ(typeid(void), linb::any().type());
  ASSERT_EQ(typeid(int), linb::any(4).type());
  ASSERT_EQ(typeid(big_type), linb::any(big_type()).type());
  ASSERT_EQ(typeid(float), linb::any(1.5f).type());
}

TEST(linb_any, exceptions) {
  bool except0 = false;
  bool except1 = false, except2 = false;
  bool except3 = false, except4 = false;

  try {
    linb::any_cast<int>(linb::any());
  } catch (const linb::bad_any_cast&) {
    except0 = true;
  }

  try {
    linb::any_cast<int>(linb::any(4.0f));
  } catch (const linb::bad_any_cast&) {
    except1 = true;
  }

  try {
    linb::any_cast<float>(linb::any(4.0f));
  } catch (const linb::bad_any_cast&) {
    except2 = true;
  }

  try {
    linb::any_cast<float>(linb::any(big_type()));
  } catch (const linb::bad_any_cast&) {
    except3 = true;
  }

  try {
    linb::any_cast<big_type>(linb::any(big_type()));
  } catch (const linb::bad_any_cast&) {
    except4 = true;
  }

  ASSERT_TRUE(except0);
  ASSERT_TRUE(except1);
  ASSERT_FALSE(except2);
  ASSERT_TRUE(except3);
  ASSERT_FALSE(except4);
}

static bool big_check(const struct big_type& in) {
  return in.value.size() == 300 && in.value.front() == 'b' &&
         in.value.back() == 'b' && in.i_wanna_be_big[0] == 'k' &&
         in.i_wanna_be_big[50] == 'k';
}

TEST(linb_any, any_cast) {
  linb::any i4 = 4;
  linb::any i5 = 5;
  linb::any f6 = 6.0f;
  linb::any big1 = big_type();
  linb::any big2 = big_type();
  linb::any big3 = big_type();

  ASSERT_NE(nullptr, linb::any_cast<int>(&i4));
  ASSERT_EQ(nullptr, linb::any_cast<float>(&i4));
  ASSERT_EQ(5, linb::any_cast<int>(i5));
  ASSERT_EQ(6.0f, linb::any_cast<float>(f6));

  ASSERT_TRUE(big_check(linb::any_cast<big_type>(big1)));
  ASSERT_TRUE(big_check(linb::any_cast<big_type>(big2)));
  ASSERT_TRUE(big_check(linb::any_cast<big_type>(big3)));
}

template <size_t N>
struct words {
  void* w[N];
};

// small type which has nothrow move ctor but throw copy ctor
struct regression1_type {
  const void* confuse_stack_storage = reinterpret_cast<void*>(0);
  regression1_type() {}
  regression1_type(const regression1_type&) {}
  regression1_type(regression1_type&&) noexcept {}
  regression1_type& operator=(const regression1_type&) { return *this; }
  regression1_type& operator=(regression1_type&&) { return *this; }
};

TEST(linb_any, check_stack_allocation) {
  auto is_stack_allocated = [](const linb::any& a, const void* obj1) {
    uintptr_t a_ptr = (uintptr_t)(&a);
    uintptr_t obj = (uintptr_t)(obj1);
    return (obj >= a_ptr && obj < a_ptr + sizeof(linb::any));
  };

  static_assert(sizeof(std::shared_ptr<big_type>) <= sizeof(void*) * 2,
                "shared_ptr too big");

  linb::any i = 400;
  linb::any f = 400.0f;
  linb::any shared = std::shared_ptr<big_type>();
  linb::any rawptr = reinterpret_cast<void*>(0);
  linb::any big = big_type();
  linb::any w2 = words<2>();
  linb::any w3 = words<3>();

  ASSERT_TRUE(is_stack_allocated(i, linb::any_cast<int>(&i)));
  ASSERT_TRUE(is_stack_allocated(f, linb::any_cast<float>(&f)));
  ASSERT_TRUE(is_stack_allocated(rawptr, linb::any_cast<void*>(&rawptr)));
  ASSERT_TRUE(is_stack_allocated(
      shared, linb::any_cast<std::shared_ptr<big_type>>(&shared)));
  ASSERT_TRUE(!is_stack_allocated(big, linb::any_cast<big_type>(&big)));
  ASSERT_TRUE(is_stack_allocated(w2, linb::any_cast<words<2>>(&w2)));
  ASSERT_TRUE(!is_stack_allocated(w3, linb::any_cast<words<3>>(&w3)));

  // Regression test for GitHub Issue
  // https://github.com/thelink2012/any/issues/1
  linb::any r1 = regression1_type();
  ASSERT_TRUE(
      is_stack_allocated(r1, linb::any_cast<const regression1_type>(&r1)));
}
