XML=                 \
 universalphy.xml    \
 antennaprofiles.xml \
 antennapattern.xml  \
 blockagepattern.xml


EXTRA_DIST=             \
 universalphy.xml.in    \
 antennaprofiles.xml.in \
 antennapattern.xml.in  \
 blockagepattern.xml.in

edit = sed -e 's|@datadir[@]|$(pkgdatadir)|g'  

all-local:	$(XML)

universalphy.xml:	universalphy.xml.in
	$(edit) universalphy.xml.in >	universalphy.xml

antennaprofiles.xml:	antennaprofiles.xml.in
	$(edit) antennaprofiles.xml.in >	antennaprofiles.xml

antennapattern.xml:	antennapattern.xml.in
	$(edit) antennapattern.xml.in >	antennapattern.xml

blockagepattern.xml:	blockagepattern.xml.in
	$(edit) blockagepattern.xml.in >	blockagepattern.xml

clean-local:
	-rm -f $(XML)

install-exec-hook:      
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/models/universal/xml
	cp -f  $(XML) $(DESTDIR)$(datadir)/$(PACKAGE)/models/universal/xml

uninstall-local:
	rm -f  $(DESTDIR)$(datadir)/$(PACKAGE)/models/universal/xml/*
