# searching for source files : files in the SRCDIR directory with the SRCEXT extension.
file(
    GLOB_RECURSE
    source_files
    *.${SRCEXT}
)

add_library( auglib ${source_files})

