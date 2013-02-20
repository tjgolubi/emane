XML=                         \
 mitremobilitygenerator.xml


EXTRA_DIST=                   \
 mitremobilitygenerator.xml.in

edit = sed                                 \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

all-local:	$(XML)

mitremobilitygenerator.xml:	mitremobilitygenerator.xml.in
	$(edit) mitremobilitygenerator.xml.in >	mitremobilitygenerator.xml

clean-local:
	-rm -f $(XML)

install-exec-hook:      
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/generators/mitremobility/xml
	cp -f  $(XML) $(DESTDIR)$(datadir)/$(PACKAGE)/generators/mitremobility/xml

uninstall-local:
	rm -f  $(DESTDIR)$(datadir)/$(PACKAGE)/generators/mitremobility/xml/*
