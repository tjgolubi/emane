dist_bin_SCRIPTS =        \
 emaneloglevel            \
 emanegendeploymentxml    \
 emanegentransportxml     \
 emanegenstatisticsrequest\
 emanegenstatisticsxml    \
 emanegenxml              \
 emaneconvertdtdpath

EXTRA_DIST =               \
 emane-service             \
 emaneeventd-service       \
 emaneeventservice-service \
 emanetransportd-service   \
 emanegenxml.in

all-local:	emanegenxml

install-exec-hook:	
	$(mkinstalldirs) $(DESTDIR)$(sysconfdir)/init.d
	cp -f  emane-service $(DESTDIR)$(sysconfdir)/init.d/emane
	cp -f  emanetransportd-service $(DESTDIR)$(sysconfdir)/init.d/emanetransportd
	cp -f  emaneeventd-service $(DESTDIR)$(sysconfdir)/init.d/emaneeventd
	cp -f  emaneeventservice-service $(DESTDIR)$(sysconfdir)/init.d/emaneeventservice

uninstall-local:
	rm -f $(sysconfdir)/init.d/emane 
	rm -f $(sysconfdir)/init.d/emanetransportd
	rm -f $(sysconfdir)/init.d/emaneeventd
	rm -f $(sysconfdir)/init.d/emaneeventservice

emanegenxml:	emanegenxml.in
	sed -e 's|@datadir[@]|$(datadir)|g' emanegenxml.in > emanegenxml
	chmod +x  emanegenxml

clean-local:	
	rm -f emanegenxml
