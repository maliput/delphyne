# Originally from https://github.com/bbreslauer/qt-bazel-example .
# No explicit license on the repository, but in
# https://github.com/bbreslauer/qt-bazel-example/issues/3 ,
# the author explicitly says that an Apache 2.0 license is "fine".

load("@//tools:pathutils.bzl", "basename")

def qt_rcc_gen(name, qrc, srcs, out):
  """Compiles a QT resource file and makes a .cpp for it.

  Args:
    name: A name for the rule.
    qrc: The qrc file to compile.
    srcs: The assets to include in the qrc file.
    out: The name of the single output file.
  """
  fname_no_ext = basename(qrc).split('.')[0]
  native.genrule(
      name = "%s_qrc" % name,
      srcs = [qrc] + srcs,
      outs = [out],
      cmd = "rcc -qt=5 --name %s --output $@ $(locations %s)" % (fname_no_ext, qrc),
  )

def qt_moc_gen(name, hdr, out):
  """Generates the MOC file for a QT library

  Args:
    name: A name for the rule.
    hdr: The header file to use to generate moc
    out: The name of the single output file.
  """
  native.genrule(
      name = "%s_moc" % name,
      srcs = [hdr],
      outs = [out],
      cmd =  "moc -qt=5 $(location %s) -o $@" % (hdr),
  )
