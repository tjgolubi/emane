XML=                \
 eventservice.xml


EXTRA_DIST=          \
 eventservice.xml.in

edit = sed                                 \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

all-local:	$(XML)

eventservice.xml:	eventservice.xml.in
	$(edit) eventservice.xml.in >	eventservice.xml

clean-local:
	-rm -f $(XML)

install-exec-hook:      
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/services/eventserver/xml
	cp -f  $(XML) $(DESTDIR)$(datadir)/$(PACKAGE)/services/eventserver/xml

uninstall-local:
	rm -f  $(DESTDIR)$(datadir)/$(PACKAGE)/services/eventserver/xml/*

