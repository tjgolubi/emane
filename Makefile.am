ACLOCAL_AMFLAGS = -I m4

SUBDIRS= src/libemanexmlparser          \
         agents/sampleagent             \
         generators/antennaprofile      \
         generators/antennaprofile/xml  \
         generators/mitremobility       \
         generators/mitremobility/xml   \
         generators/emulationscript     \
         generators/emulationscript/xml \
         transports/common              \
         transports/virtual             \
         transports/virtual/xml         \
         src/libemane                   \
         src/emane                      \
         src/emanetransportd            \
         src/emanetransportd/xml        \
         src/emaneeventd                \
         src/emaneeventd/xml            \
         src/emaneeventservice          \
         src/emaneeventservice/xml      \
         models/bypass/maclayer         \
         models/bypass/phylayer         \
         models/bypass/xml              \
         models/ieee80211abg/maclayer   \
         models/ieee80211abg/xml        \
         models/rfpipe/maclayer         \
         models/rfpipe/xml              \
         models/universal/phylayer      \
         models/universal/xml           \
         shims/timinganalysis/src       \
         shims/timinganalysis/xml       \
         shims/phyapitest/src           \
         shims/phyapitest/xml           \
         include/libemane               \
         include/emane                  \
         include/emaneutils             \
         include/emaneevents            \
         include/emanecontrolmessages   \
         scripts                        \
         scripts/doc                    \
         dtd                            \
         doc                            

if EMANE_HAS_PTY
SUBDIRS += agents/gpsdlocation          \
           agents/gpsdlocation/xml
endif

edit = sed                              \
       -e 's|@VERSION[@]|$(VERSION)|g'

if EMANE_HAS_PCAP
SUBDIRS += transports/raw               \
           transports/raw/xml
endif

dist-hook: emane.spec 

cleantar:
	@(rm -f emane*.tar.gz)


if HAVE_DEB
cleandeb:
	dh_clean
else
cleandeb:
endif

RPMARCH=`rpm --showrc | awk '/^build arch/ {print $$4}'`

if HAVE_RPMBUILD

rpm:	clean-local dist-gzip
	mkdir -p .rpmbuild/BUILD \
	.rpmbuild/SPECS \
	.rpmbuild/SOURCES \
	.rpmbuild/SRPMS \
	.rpmbuild/RPMS/$(RPMARCH) \
	.rpmbuild/tmp
	cp -f $(PACKAGE)-$(VERSION).tar.gz .rpmbuild/SOURCES
	rpmbuild --clean -bb $(top_srcdir)/emane.spec --define "_topdir $$PWD/.rpmbuild" --define "_tmppath $$PWD/.rpmbuild/tmp"
	@echo "============================================================"
	@echo "RPMs located in .rpmbuild/RPMS and .rpmbuild/SRPMS"

endif

clean-local:	cleantar cleandeb
	-rm -rf .rpmbuild

if HAVE_DEB
deb: debian/changelog
	dpkg-buildpackage -b -us -uc
endif

if HAVE_DOXYGEN

doxygen:
	$(MAKE) -C doc doxygen

endif

debian/%:	debian/%.in
	if test -f $@; then chmod u+w $@; fi
	$(edit) $< > $@
	chmod g-w,u-w $@

EXTRA_DIST =         \
 AUTHORS             \
 COPYING             \
 ChangeLog           \
 NEWS                \
 emane.spec.in       \
 emane.spec          \
 PROBLEM-REPORT-FORM \
 SPONSOR             \
 UNLIMITED-RIGHTS

DISTCLEANFILES = .rpmbuild debian/changelog
