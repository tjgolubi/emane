lib_LTLIBRARIES = libemanexmlparser.la

INCLUDES =                           \
 -I@top_srcdir@/include

libemanexmlparser_la_CPPFLAGS= \
 $(LIBXML2_CFLAGS)             \
 $(ACE_CFLAGS)              \
 $(AM_CFLAGS)

libemanexmlparser_la_SOURCES=   \
 configurationparser.cc         \
 platformconfiguration.cc       \
 layerconfiguration.cc          \
 nemconfiguration.cc            \
 phylayerconfiguration.cc       \
 maclayerconfiguration.cc       \
 shimlayerconfiguration.cc      \
 transportlayerconfiguration.cc

EXTRA_DIST=                     \
 configurationparser.h          \
 platformconfiguration.h        \
 layerconfiguration.h           \
 nemconfiguration.h             \
 phylayerconfiguration.h        \
 maclayerconfiguration.h        \
 shimlayerconfiguration.h       \
 transportlayerconfiguration.h  \
 emaneparseexception.h         

libemanexmlparser_la_LIBADD=                 \
 $(WIN32_EXTRA_LIBADD)

libemanexmlparser_la_LDFLAGS= \
 $(WIN32_EXTRA_LDFLAGS)       \
 $(LIBXML2_LIBS)              \
 $(ACE_LIBS)             
