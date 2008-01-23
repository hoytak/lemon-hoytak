EXTRA_DIST += \
	doc/Makefile \
	doc/Doxyfile.in \
	doc/coding_style.dox \
	doc/dirs.dox \
	doc/groups.dox \
	doc/license.dox \
	doc/mainpage.dox \
	doc/namespaces.dox \
	doc/html

doc/html:
	$(MAKE) $(AM_MAKEFLAGS) html

html-local:
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
