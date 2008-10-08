EXTRA_DIST += \
	doc/Doxyfile.in \
	doc/coding_style.dox \
	doc/dirs.dox \
	doc/groups.dox \
	doc/lgf.dox \
	doc/license.dox \
	doc/mainpage.dox \
	doc/migration.dox \
	doc/named-param.dox \
	doc/namespaces.dox \
	doc/html \
	doc/CMakeLists.txt

DOC_EPS_IMAGES18 = \
	nodeshape_0.eps \
	nodeshape_1.eps \
	nodeshape_2.eps \
	nodeshape_3.eps \
	nodeshape_4.eps

DOC_EPS_IMAGES = \
	$(DOC_EPS_IMAGES18)

DOC_PNG_IMAGES = \
	$(DOC_EPS_IMAGES:%.eps=doc/gen-images/%.png)

EXTRA_DIST += $(DOC_EPS_IMAGES:%=doc/images/%)

doc/html:
	$(MAKE) $(AM_MAKEFLAGS) html

GS_COMMAND=gs -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4

$(DOC_EPS_IMAGES18:%.eps=doc/gen-images/%.png): doc/gen-images/%.png: doc/images/%.eps
	-mkdir doc/gen-images
	if test ${gs_found} = yes; then \
	  $(GS_COMMAND) -sDEVICE=pngalpha -r18 -sOutputFile=$@ $<; \
	else \
	  echo; \
	  echo "Ghostscript not found."; \
	  echo; \
	  exit 1; \
	fi

html-local: $(DOC_PNG_IMAGES)
	if test ${doxygen_found} = yes; then \
	  cd doc; \
	  doxygen Doxyfile; \
	  cd ..; \
	else \
	  echo; \
	  echo "Doxygen not found."; \
	  echo; \
	  exit 1; \
	fi

clean-local:
	-rm -rf doc/html
	-rm -f doc/doxygen.log
	-rm -f $(DOC_PNG_IMAGES)
	-rm -rf doc/gen-images

update-external-tags:
	wget -O doc/libstdc++.tag.tmp http://gcc.gnu.org/onlinedocs/libstdc++/latest-doxygen/libstdc++.tag && \
	mv doc/libstdc++.tag.tmp doc/libstdc++.tag || \
	rm doc/libstdc++.tag.tmp

install-html-local: doc/html
	@$(NORMAL_INSTALL)
	$(mkinstalldirs) $(DESTDIR)$(htmldir)/docs
	for p in doc/html/*.{html,css,png,map,gif,tag} ; do \
	  f="`echo $$p | sed -e 's|^.*/||'`"; \
	  echo " $(INSTALL_DATA) $$p $(DESTDIR)$(htmldir)/docs/$$f"; \
	  $(INSTALL_DATA) $$p $(DESTDIR)$(htmldir)/docs/$$f; \
	done

uninstall-local:
	@$(NORMAL_UNINSTALL)
	for p in doc/html/*.{html,css,png,map,gif,tag} ; do \
	  f="`echo $$p | sed -e 's|^.*/||'`"; \
	  echo " rm -f $(DESTDIR)$(htmldir)/docs/$$f"; \
	  rm -f $(DESTDIR)$(htmldir)/docs/$$f; \
	done

.PHONY: update-external-tags
