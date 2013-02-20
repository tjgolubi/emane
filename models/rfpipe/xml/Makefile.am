XML=            \
 rfpipenem.xml  \
 rfpipemac.xml  \
 rfpipepcr.xml


EXTRA_DIST=        \
 rfpipenem.xml.in  \
 rfpipemac.xml.in  \
 rfpipepcr.xml.in

edit = sed                                 \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

all-local:	$(XML)

rfpipenem.xml:	rfpipenem.xml.in
	$(edit) rfpipenem.xml.in >	rfpipenem.xml

rfpipemac.xml:	rfpipemac.xml.in
	$(edit) rfpipemac.xml.in >	rfpipemac.xml

rfpipepcr.xml:	rfpipepcr.xml.in
	$(edit) rfpipepcr.xml.in >	rfpipepcr.xml

clean-local:
	-rm -f $(XML)

install-exec-hook:      
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/models/rfpipe/xml
	cp -f  $(XML) $(DESTDIR)$(datadir)/$(PACKAGE)/models/rfpipe/xml

uninstall-local:
	rm -f  $(DESTDIR)$(datadir)/$(PACKAGE)/models/rfpipe/xml/*
