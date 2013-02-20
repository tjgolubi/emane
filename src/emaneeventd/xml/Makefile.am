XML=                \
 eventdaemon.xml


EXTRA_DIST=          \
 eventdaemon.xml.in

edit = sed                                 \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

all-local:	$(XML)

eventdaemon.xml:	eventdaemon.xml.in
	$(edit) eventdaemon.xml.in >	eventdaemon.xml

clean-local:
	-rm -f $(XML)

install-exec-hook:      
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/src/emaneeventd/xml
	cp -f  $(XML) $(DESTDIR)$(datadir)/$(PACKAGE)/src/emaneeventd/xml

uninstall-local:
	rm -f  $(DESTDIR)$(datadir)/$(PACKAGE)/src/emaneeventd/xml/*

