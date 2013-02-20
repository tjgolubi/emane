XML=                \
 transvirtual.xml


EXTRA_DIST=          \
 transvirtual.xml.in

edit = sed                                 \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

all-local:	$(XML)

transvirtual.xml:	transvirtual.xml.in
	$(edit) transvirtual.xml.in >	transvirtual.xml

clean-local:
	-rm -f $(XML)

install-exec-hook:      
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/transports/virtual/xml
	cp -f  $(XML) $(DESTDIR)$(datadir)/$(PACKAGE)/transports/virtual/xml

uninstall-local:
	rm -f  $(DESTDIR)$(datadir)/$(PACKAGE)/transports/virtual/xml/*

