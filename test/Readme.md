# Tests

## Running Tests (Local)

```
$ ./test/run_tests.sh
```

## Errata

### Categories

Regression, performance and integration. If we get so many tests, might be worth
breaking them further down by functionality (e.g. python bindings tests).

### Useful Tools

There was a header file with a bag of tricks for various things that are
useful to do in tests - paths, filesystem, number generators. Not using
it now, but it's in the history if we want to pull it back. If we do,
perhaps lump it with the test helper library.

### Test Libraries

Similar to above, there's a test_helpers library. Probably worth
breaking that one at some point out to the library folder and having
a dedicated library available to assist with testing delphyne and delphyne_gui.

