XML=                         \
 antennaprofilegenerator.xml


EXTRA_DIST=                   \
 antennaprofilegenerator.xml.in

edit = sed                                 \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

all-local:	$(XML)

antennaprofilegenerator.xml:	antennaprofilegenerator.xml.in
	$(edit) antennaprofilegenerator.xml.in >	antennaprofilegenerator.xml

clean-local:
	-rm -f $(XML)

install-exec-hook:      
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/generators/antennaprofile/xml
	cp -f  $(XML) $(DESTDIR)$(datadir)/$(PACKAGE)/generators/antennaprofile/xml

uninstall-local:
	rm -f  $(DESTDIR)$(datadir)/$(PACKAGE)/generators/antennaprofile/xml/*
