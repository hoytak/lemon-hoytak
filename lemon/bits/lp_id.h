/* -*- mode: C++; indent-tabs-mode: nil; -*-
 *
 * This file is a part of LEMON, a generic C++ optimization library.
 *
 * Copyright (C) 2003-2008
 * Egervary Jeno Kombinatorikus Optimalizalasi Kutatocsoport
 * (Egervary Research Group on Combinatorial Optimization, EGRES).
 *
 * Permission to use, modify and distribute this software is granted
 * provided that this copyright notice appears in all copies. For
 * precise terms see the accompanying LICENSE file.
 *
 * This software is provided "AS IS" with no warranty of any kind,
 * express or implied, and with no claim as to its suitability for any
 * purpose.
 *
 */

#ifndef LEMON_BITS_LP_SOLVER_ID_H
#define LEMON_BITS_LP_SOLVER_ID_H

namespace lemon {

  namespace _lp_bits {

    struct LpIdImpl {
      std::vector<int> index;
      std::vector<int> cross;
      int first_index;
      int first_free;
    };

    class LpId {
    public:

      class IdHandler {
      public:
        virtual ~IdHandler() {}
        virtual int addId(LpIdImpl&) = 0;
        virtual void eraseId(LpIdImpl&, int xn) = 0;
      };

      LpId(int min_index = 0) {
        id_handler = 0;
        impl.first_free = -1;
        impl.first_index = min_index;
        impl.cross.resize(impl.first_index);
      }

      LpId(const LpId& li) {
        id_handler = 0;
        impl = li.impl;
      }

      LpId& operator=(const LpId& li) {
        id_handler = 0;
        impl = li.impl;
        return *this;
      }

      void setIdHandler(IdHandler& ih) {
        id_handler = &ih;
      }

      int fixId(int fn) const {return impl.cross[fn];}
      int floatingId(int xn) const {return impl.index[xn];}

      int addId() {
        if (id_handler == 0) {
          int xn, fn = impl.cross.size();
          if (impl.first_free == -1) {
            xn = impl.index.size();
            impl.index.push_back(fn);
          } else {
            xn = impl.first_free;
            impl.first_free = impl.index[impl.first_free];
            impl.index[xn] = fn;
          }
          impl.cross.push_back(xn);
          return xn;
        } else {
          return id_handler->addId(impl);
        }
      }

      void eraseId(int xn) {
        if (id_handler == 0) {
          int fn = impl.index[xn];
          impl.index[xn] = impl.first_free;
          impl.first_free = xn;
          for(int i = fn + 1; i < int(impl.cross.size()); ++i) {
            impl.cross[i - 1] = impl.cross[i];
            impl.index[impl.cross[i]]--;
          }
          impl.cross.pop_back();
        } else {
          id_handler->eraseId(impl, xn);
        }
      }

      void firstFloating(int& fn) const {
        fn = impl.first_index;
        if (fn == int(impl.cross.size())) fn = -1;
      }

      void nextFloating(int& fn) const {
        ++fn;
        if (fn == int(impl.cross.size())) fn = -1;
      }

      void firstFix(int& xn) const {
        int fn;
        firstFloating(fn);
        xn = fn != -1 ? fixId(fn) : -1;
      }

      void nextFix(int& xn) const {
        int fn = floatingId(xn);
        nextFloating(fn);
        xn = fn != -1 ? fixId(fn) : -1;
      }

    protected:
      LpIdImpl impl;
      IdHandler *id_handler;
    };

    class RelocateIdHandler : public LpId::IdHandler {
    public:

      virtual int addId(LpIdImpl& impl) {
        int xn, fn = impl.cross.size();
        if (impl.first_free == -1) {
          xn = impl.index.size();
          impl.index.push_back(fn);
        } else {
          xn = impl.first_free;
          impl.first_free = impl.index[impl.first_free];
          impl.index[xn] = fn;
        }
        impl.cross.push_back(xn);
        return xn;
      }

      virtual void eraseId(LpIdImpl& impl, int xn) {
        int fn = impl.index[xn];
        impl.index[xn] = impl.first_free;
        impl.first_free = xn;
        impl.cross[fn] = impl.cross.back();
        impl.index[impl.cross.back()] = fn;
        impl.cross.pop_back();
      }
    };
  }
}

#endif
