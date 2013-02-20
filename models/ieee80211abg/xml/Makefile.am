XML=                   \
 ieee80211abgnem.xml   \
 ieee80211abgmac.xml   \
 ieee80211pcr.xml


EXTRA_DIST=               \
 ieee80211abgnem.xml.in   \
 ieee80211abgmac.xml.in   \
 ieee80211pcr.xml.in 

edit = sed                                 \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

all-local:	$(XML)

ieee80211abgnem.xml:	ieee80211abgnem.xml.in
	$(edit) ieee80211abgnem.xml.in >	ieee80211abgnem.xml

ieee80211abgmac.xml:	ieee80211abgmac.xml.in
	$(edit) ieee80211abgmac.xml.in >	ieee80211abgmac.xml

ieee80211pcr.xml:	ieee80211pcr.xml.in
	$(edit) ieee80211pcr.xml.in >	ieee80211pcr.xml

clean-local:
	-rm -f $(XML)

install-exec-hook:      
	$(mkinstalldirs) $(DESTDIR)$(datadir)/$(PACKAGE)/models/ieee80211abg/xml
	cp -f  $(XML) $(DESTDIR)$(datadir)/$(PACKAGE)/models/ieee80211abg/xml

uninstall-local:
	rm -f  $(DESTDIR)$(datadir)/$(PACKAGE)/models/ieee80211abg/xml/*
