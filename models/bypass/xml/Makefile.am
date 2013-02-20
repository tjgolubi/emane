XML=            \
 bypassnem.xml  \
 bypassmac.xml  \
 bypassphy.xml  


EXTRA_DIST=        \
 bypassnem.xml.in  \
 bypassmac.xml.in  \
 bypassphy.xml.in  

edit = sed                                 \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

all-local:	$(XML)

bypassnem.xml:	bypassnem.xml.in
	$(edit) bypassnem.xml.in >	bypassnem.xml

bypassmac.xml:	bypassmac.xml.in
	$(edit) bypassmac.xml.in >	bypassmac.xml

bypassphy.xml:	bypassphy.xml.in
	$(edit) bypassphy.xml.in >	bypassphy.xml

clean-local:
	-rm -f $(XML)

install-exec-hook:      
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/models/bypass/xml
	cp -f  $(XML) $(DESTDIR)$(datadir)/$(PACKAGE)/models/bypass/xml

uninstall-local:
	rm -f  $(DESTDIR)$(datadir)/$(PACKAGE)/models/bypass/xml/*
