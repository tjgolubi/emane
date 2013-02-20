XML=                        \
 phyapitestshim.xml


EXTRA_DIST=                 \
 phyapitestshim.xml.in

edit = sed                                 \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

all-local:	$(XML)

phyapitestshim.xml:	phyapitestshim.xml.in
	$(edit) phyapitestshim.xml.in >	phyapitestshim.xml

clean-local:
	-rm -f $(XML)

install-exec-hook:      
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/shims/phyapitest/xml
	cp -f  $(XML) $(DESTDIR)$(datadir)/$(PACKAGE)/shims/phyapitest/xml

uninstall-local:
	rm -f  $(DESTDIR)$(datadir)/$(PACKAGE)/shims/phyapitest/xml/*

