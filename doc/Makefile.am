DOCNAME=emane

man_MANS =                  \
 emane.1                    \
 emanetransportd.1          \
 emaneeventd.1              \
 emaneeventservice.1

EXTRA_DIST=                 \
 emane.doxygen.in           \
 emane.1.in                 \
 emaneeventd.1.in           \
 emanetransportd.1.in       \
 emaneeventservice.1.in     

DISTCLEANFILES = 

edit = sed                                 \
       -e 's|@datadir[@]|$(pkgdatadir)|g'  \
       -e 's|@VERSION[@]|$(VERSION)|g'     \
       -e 's|@RELDATE[@]|$(RELDATE)|g'

all-local:

dist-hook: emane.doxygen 

doxygen:
	@doxygen emane.doxygen

emane.1:	emane.1.in
	if test -f $@; then chmod u+w $@; fi
	$(edit) $< > $@
	chmod g-w,u-w $@

emaneeventd.1:	emaneeventd.1.in
	if test -f $@; then chmod u+w $@; fi
	$(edit) $< > $@
	chmod g-w,u-w $@

emanetransportd.1:	emanetransportd.1.in
	if test -f $@; then chmod u+w $@; fi
	$(edit) $< > $@
	chmod g-w,u-w $@

emaneeventservice.1:	emaneeventservice.1.in
	if test -f $@; then chmod u+w $@; fi
	$(edit) $< > $@
	chmod g-w,u-w $@

clean-local:
	-rm -f  $(man_MANS)

