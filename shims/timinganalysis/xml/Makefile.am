XML=                        \
 timinganalysisshim.xml


EXTRA_DIST=                 \
 timinganalysisshim.xml.in

edit = sed                                 \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

all-local:	$(XML)

timinganalysisshim.xml:	timinganalysisshim.xml.in
	$(edit) timinganalysisshim.xml.in >	timinganalysisshim.xml

clean-local:
	-rm -f $(XML)
