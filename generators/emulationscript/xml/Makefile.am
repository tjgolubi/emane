XML=                         \
 emulationscriptgenerator.xml


EXTRA_DIST=                      \
 emulationscriptgenerator.xml.in \
 EmulationScriptSchema.xsd

edit = sed                                 \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

all-local:	$(XML)

emulationscriptgenerator.xml:	emulationscriptgenerator.xml.in
	$(edit) emulationscriptgenerator.xml.in >	emulationscriptgenerator.xml

clean-local:
	-rm -f $(XML)

install-exec-hook:      
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/generators/emulationscript/xml
	cp -f  $(XML) $(DESTDIR)$(datadir)/$(PACKAGE)/generators/emulationscript/xml

uninstall-local:
	rm -f  $(DESTDIR)$(datadir)/$(PACKAGE)/generators/emulationscript/xml/*
