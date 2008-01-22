htmldir = $(datadir)/doc/$(PACKAGE)/html

EXTRA_DIST += \
	doc/Makefile \
	doc/Doxyfile.in

doc:
	if test ${doxygen_found} = yes; then \
	  cd doc; \
	  doxygen Doxyfile; \
	  cd ..; \
	fi

doc-clean:
	if test ${doxygen_found} = yes; then \
	  rm -rf doc/html; \
	  rm -f doc/doxygen.log; \
	  cd doc; \
	  doxygen Doxyfile; \
	  cd ..; \
	fi

clean-local:
	-rm -rf doc/html
	-rm -f doc/doxygen.log

doc/html:
	$(MAKE) $(AM_MAKEFLAGS) doc-clean

update-external-tags:
	wget -O doc/libstdc++.tag.tmp http://gcc.gnu.org/onlinedocs/libstdc++/latest-doxygen/libstdc++.tag && \
	mv doc/libstdc++.tag.tmp doc/libstdc++.tag || \
	rm doc/libstdc++.tag.tmp

if WANT_DOC

install-data-local: doc/html
	@$(NORMAL_INSTALL)
	$(mkinstalldirs) $(DESTDIR)$(htmldir)
	@dir='doc/html'; shopt -s nullglob; for p in $$dir/*.html $$dir/*.css $$dir/*.png $$dir/*.gif $$dir/*.dot $$dir/*.php $$dir/*.idx $$dir/*.tag ; do \
	  f="`echo $$p | sed -e 's|^.*/||'`"; \
	  echo " $(INSTALL_DATA) $$p $(DESTDIR)$(htmldir)/$$f"; \
	  $(INSTALL_DATA) $$p $(DESTDIR)$(htmldir)/$$f; \
	done

uninstall-local: doc/html
	@$(NORMAL_UNINSTALL)
	@dir='doc/html'; shopt -s nullglob; for p in $$dir/*.html $$dir/*.css $$dir/*.png $$dir/*.gif $$dir/*.dot $$dir/*.php $$dir/*.idx $$dir/*.tag ; do \
	  f="`echo $$p | sed -e 's|^.*/||'`"; \
	  echo " rm -f $(DESTDIR)$(htmldir)/$$f"; \
	  rm -f $(DESTDIR)$(htmldir)/$$f; \
	done

endif WANT_DOC

.PHONY: doc doc-clean
