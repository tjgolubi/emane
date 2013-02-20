XML=           \
 transraw.xml


EXTRA_DIST=     \
 transraw.xml.in

edit = sed                                 \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

all-local:	$(XML)

transraw.xml:	transraw.xml.in
	$(edit) transraw.xml.in >	transraw.xml

clean-local:
	-rm -f $(XML)

install-exec-hook:      
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/transports/raw/xml
	cp -f  $(XML) $(DESTDIR)$(datadir)/$(PACKAGE)/transports/raw/xml

uninstall-local:
	rm -f  $(DESTDIR)$(datadir)/$(PACKAGE)/transports/raw/xml/*

