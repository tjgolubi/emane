man_MANS =                  \
 emanegentransportxml.1     \
 emaneloglevel.1            \
 emanegendeploymentxml.1    \
 emanegenstatisticsrequest.1\
 emanegenstatisticsxml.1    \
 emanegenxml.1

EXTRA_DIST =                    \
 emanegendeploymentxml.1.in     \
 emanegenstatisticsrequest.1.in \
 emanegenstatisticsxml.1.in     \
 emanegentransportxml.1.in      \
 emanegenxml.1.in               \
 emaneloglevel.1.in

edit = sed                                 \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  \
       -e 's|@VERSION[@]|$(VERSION)|g'     \
       -e 's|@RELDATE[@]|$(RELDATE)|g'

all-local:	$(man_MANS)

emanegentransportxml.1:	emanegentransportxml.1.in
	$(edit) $< > $@

emaneloglevel.1:	emaneloglevel.1.in
	$(edit) $< > $@

emanegendeploymentxml.1:	emanegendeploymentxml.1.in
	$(edit) $< > $@

emanegenstatisticsrequest.1:	emanegenstatisticsrequest.1.in
	$(edit) $< > $@

emanegenstatisticsxml.1:	emanegenstatisticsxml.1.in
	$(edit) $< > $@

emanegenxml.1:	emanegenxml.1.in
	$(edit) $< > $@

clean-local:
	rm -f $(man_MANS)

