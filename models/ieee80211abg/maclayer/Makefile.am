lib_LTLIBRARIES = libieee80211abgmaclayer.la

INCLUDES =              \
 -I@top_srcdir@/include \
 -I../common 

libieee80211abgmaclayer_la_CPPFLAGS= \
 $(ACE_CFLAGS)                       \
 $(AM_CFLAGS)                        \
 $(LIBXML2_CFLAGS)

libieee80211abgmaclayer_la_SOURCES = \
 ieee80211abgmaclayer.cc             \
 modetimingparameters.cc             \
 neighborentry.cc                    \
 neighbor2hopentry.cc                \
 neighbormanager.cc                  \
 macconfig.cc                        \
 pcrmanager.cc                       \
 macstatistics.cc                    \
 downstreamqueue.cc                  \
 transmissiontxstate.cc              \
 idletxstate.cc                      \
 multicasttxstate.cc                 \
 multicastpretxstate.cc              \
 multicastposttxstate.cc             \
 unicasttxstate.cc                   \
 unicastpretxstate.cc                \
 unicastposttxstate.cc               \
 unicastrtsctstxstate.cc             \
 unicastrtsctspretxstate.cc          \
 unicastrtsctsposttxstate.cc         \
 collisiontable.cc

EXTRA_DIST=                          \
 downstreamqueue.h                   \
 neighborentry.h                     \
 neighbor2hopentry.h                 \
 neighbormanager.h                   \
 macconfig.h                         \
 pcrmanager.h                        \
 ieee80211abgmacheader.h             \
 ieee80211abgmaclayer.h              \
 macstatistics.h                     \
 idletxstate.h                       \
 modetimingparameters.h              \
 multicasttxstate.h                  \
 multicastpretxstate.h               \
 multicastposttxstate.h              \
 transmissiontxstate.h               \
 unicasttxstate.h                    \
 unicastpretxstate.h                 \
 unicastposttxstate.h                \
 unicastrtsctstxstate.h              \
 unicastrtsctspretxstate.h           \
 unicastrtsctsposttxstate.h          \
 utils.h                             \
 neighbortype.h                      \
 neighbortype.inl                    \
 onehopnbrlistevent.h                \
 onehopnbrlistevent.inl              \
 collisiontable.h                    \
 msgtypes.h


libieee80211abgmaclayer_la_LIBADD= \
 $(WIN32_EXTRA_LIBADD)

libieee80211abgmaclayer_la_LDFLAGS= \
 $(WIN32_EXTRA_LDFLAGS)             \
 $(ACE_LIBS)                        \
 $(LIBXML2_LIBS)
