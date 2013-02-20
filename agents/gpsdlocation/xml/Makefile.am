XML=                         \
 gpsdlocationagent.xml


EXTRA_DIST=                   \
 gpsdlocationagent.xml.in

edit = sed                                 \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

all-local:	$(XML)

gpsdlocationagent.xml:	gpsdlocationagent.xml.in
	$(edit) gpsdlocationagent.xml.in >	gpsdlocationagent.xml

clean-local:
	-rm -f $(XML)

install-exec-hook:      
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/agents/gpsdlocation/xml
	cp -f  $(XML) $(DESTDIR)$(datadir)/$(PACKAGE)/agents/gpsdlocation/xml

uninstall-local:
	rm -f  $(DESTDIR)$(datadir)/$(PACKAGE)/agents/gpsdlocation/xml/*

