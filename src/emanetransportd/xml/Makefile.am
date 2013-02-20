XML=                \
 transportdaemon.xml


EXTRA_DIST=          \
 transportdaemon.xml.in

edit = sed                                 \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  

all-local:	$(XML)

transportdaemon.xml:	transportdaemon.xml.in
	$(edit) transportdaemon.xml.in >	transportdaemon.xml

clean-local:
	-rm -f $(XML)
