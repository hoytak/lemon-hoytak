/* -*- C++ -*-
 *
 * This file is a part of LEMON, a generic C++ optimization library
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

#ifndef LEMON_TIME_MEASURE_H
#define LEMON_TIME_MEASURE_H

///\ingroup timecount
///\file
///\brief Tools for measuring cpu usage

#include <sys/times.h>

#include <sys/time.h>
#include <fstream>
#include <iostream>
#include <unistd.h>

namespace lemon {

  /// \addtogroup timecount
  /// @{

  /// A class to store (cpu)time instances.

  /// This class stores five time values.
  /// - a real time
  /// - a user cpu time
  /// - a system cpu time
  /// - a user cpu time of children
  /// - a system cpu time of children
  ///
  /// TimeStamp's can be added to or substracted from each other and
  /// they can be pushed to a stream.
  ///
  /// In most cases, perhaps the \ref Timer or the \ref TimeReport
  /// class is what you want to use instead.
  ///
  ///\author Alpar Juttner

  class TimeStamp
  {
    struct rtms 
    {
      double tms_utime;
      double tms_stime;
      double tms_cutime;
      double tms_cstime;
      rtms() {}
      rtms(tms ts) : tms_utime(ts.tms_utime), tms_stime(ts.tms_stime),
		     tms_cutime(ts.tms_cutime), tms_cstime(ts.tms_cstime) {}
    };
    rtms ts;
    double real_time;
  
    rtms &getTms() {return ts;}
    const rtms &getTms() const {return ts;}

    void _reset() { 
      ts.tms_utime = ts.tms_stime = ts.tms_cutime = ts.tms_cstime = 0; 
      real_time = 0;
    }

  public:

    ///Read the current time values of the process
    void stamp()
    {
      timeval tv;
      tms _ts;
      times(&_ts);
      gettimeofday(&tv, 0);real_time=tv.tv_sec+double(tv.tv_usec)/1e6;
      ts=_ts;
    }
  
    /// Constructor initializing with zero
    TimeStamp()
    { _reset(); }
    ///Constructor initializing with the current time values of the process
    TimeStamp(void *) { stamp();}
  
    ///Set every time value to zero
    TimeStamp &reset() {_reset();return *this;}

    ///\e
    TimeStamp &operator+=(const TimeStamp &b)
    {
      ts.tms_utime+=b.ts.tms_utime;
      ts.tms_stime+=b.ts.tms_stime;
      ts.tms_cutime+=b.ts.tms_cutime;
      ts.tms_cstime+=b.ts.tms_cstime;
      real_time+=b.real_time;
      return *this;
    }
    ///\e
    TimeStamp operator+(const TimeStamp &b) const
    {
      TimeStamp t(*this);
      return t+=b;
    }
    ///\e
    TimeStamp &operator-=(const TimeStamp &b)
    {
      ts.tms_utime-=b.ts.tms_utime;
      ts.tms_stime-=b.ts.tms_stime;
      ts.tms_cutime-=b.ts.tms_cutime;
      ts.tms_cstime-=b.ts.tms_cstime;
      real_time-=b.real_time;
      return *this;
    }
    ///\e
    TimeStamp operator-(const TimeStamp &b) const
    {
      TimeStamp t(*this);
      return t-=b;
    }
    ///\e
    TimeStamp &operator*=(double b)
    {
      ts.tms_utime*=b;
      ts.tms_stime*=b;
      ts.tms_cutime*=b;
      ts.tms_cstime*=b;
      real_time*=b;
      return *this;
    }
    ///\e
    TimeStamp operator*(double b) const
    {
      TimeStamp t(*this);
      return t*=b;
    }
    friend TimeStamp operator*(double b,const TimeStamp &t);
    ///\e
    TimeStamp &operator/=(double b)
    {
      ts.tms_utime/=b;
      ts.tms_stime/=b;
      ts.tms_cutime/=b;
      ts.tms_cstime/=b;
      real_time/=b;
      return *this;
    }
    ///\e
    TimeStamp operator/(double b) const
    {
      TimeStamp t(*this);
      return t/=b;
    }
    ///The time ellapsed since the last call of stamp()
    TimeStamp ellapsed() const
    {
      TimeStamp t(NULL);
      return t-*this;
    }
  
    friend std::ostream& operator<<(std::ostream& os,const TimeStamp &t);
  
    ///Gives back the user time of the process
    double userTime() const
    {
      return double(ts.tms_utime)/sysconf(_SC_CLK_TCK);
    }
    ///Gives back the system time of the process
    double systemTime() const
    {
      return double(ts.tms_stime)/sysconf(_SC_CLK_TCK);
    }
    ///Gives back the user time of the process' children
    double cUserTime() const
    {
      return double(ts.tms_cutime)/sysconf(_SC_CLK_TCK);
    }
    ///Gives back the user time of the process' children
    double cSystemTime() const
    {
      return double(ts.tms_cstime)/sysconf(_SC_CLK_TCK);
    }
    ///Gives back the real time
    double realTime() const {return real_time;}
  };

  TimeStamp operator*(double b,const TimeStamp &t) 
  {
    return t*b;
  }
  
  ///Prints the time counters

  ///Prints the time counters in the following form:
  ///
  /// <tt>u: XX.XXs s: XX.XXs cu: XX.XXs cs: XX.XXs real: XX.XXs</tt>
  ///
  /// where the values are the
  /// \li \c u: user cpu time,
  /// \li \c s: system cpu time,
  /// \li \c cu: user cpu time of children,
  /// \li \c cs: system cpu time of children,
  /// \li \c real: real time.
  /// \relates TimeStamp
  inline std::ostream& operator<<(std::ostream& os,const TimeStamp &t)
  {
    long cls = sysconf(_SC_CLK_TCK);
    os << "u: " << double(t.getTms().tms_utime)/cls <<
      "s, s: " << double(t.getTms().tms_stime)/cls <<
      "s, cu: " << double(t.getTms().tms_cutime)/cls <<
      "s, cs: " << double(t.getTms().tms_cstime)/cls <<
      "s, real: " << t.realTime() << "s";
    return os;
  }

  ///Class for measuring the cpu time and real time usage of the process

  ///Class for measuring the cpu time and real time usage of the process.
  ///It is quite easy-to-use, here is a short example.
  ///\code
  /// #include<lemon/time_measure.h>
  /// #include<iostream>
  ///
  /// int main()
  /// {
  ///
  ///   ...
  ///
  ///   Timer t;
  ///   doSomething();
  ///   std::cout << t << '\n';
  ///   t.restart();
  ///   doSomethingElse();
  ///   std::cout << t << '\n';
  ///
  ///   ...
  ///
  /// }
  ///\endcode
  ///
  ///The \ref Timer can also be \ref stop() "stopped" and
  ///\ref start() "started" again, so it is possible to compute collected
  ///running times.
  ///
  ///\warning Depending on the operation system and its actual configuration
  ///the time counters have a certain (10ms on a typical Linux system)
  ///granularity.
  ///Therefore this tool is not appropriate to measure very short times.
  ///Also, if you start and stop the timer very frequently, it could lead to
  ///distorted results.
  ///
  ///\note If you want to measure the running time of the execution of a certain
  ///function, consider the usage of \ref TimeReport instead.
  ///
  ///\todo This shouldn't be Unix (Linux) specific.
  ///\sa TimeReport
  ///
  ///\author Alpar Juttner
  class Timer
  {
    int _running; //Timer is running iff _running>0; (_running>=0 always holds)
    TimeStamp start_time; //This is the relativ start-time if the timer
                          //is _running, the collected _running time otherwise.
    
    void _reset() {if(_running) start_time.stamp(); else start_time.reset();}
  
  public: 
    ///Constructor.

    ///\param run indicates whether or not the timer starts immediately.
    ///
    Timer(bool run=true) :_running(run) {_reset();}

    ///\name Control the state of the timer
    ///Basically a Timer can be either running or stopped,
    ///but it provides a bit finer control on the execution.
    ///The \ref Timer also counts the number of \ref start()
    ///executions, and is stops only after the same amount (or more)
    ///\ref stop() "stop()"s. This can be useful e.g. to compute the running time
    ///of recursive functions.
    ///

    ///@{

    ///Reset and stop the time counters

    ///This function resets and stops the time counters
    ///\sa restart()
    void reset()
    {
      _running=0;
      _reset();
    }

    ///Start the time counters
    
    ///This function starts the time counters.
    ///
    ///If the timer is started more than ones, it will remain running
    ///until the same amount of \ref stop() is called.
    ///\sa stop()
    void start() 
    {
      if(_running) _running++;
      else {
	_running=1;
	TimeStamp t;
	t.stamp();
	start_time=t-start_time;
      }
    }

    
    ///Stop the time counters

    ///This function stops the time counters. If start() was executed more than
    ///once, then the same number of stop() execution is necessary the really
    ///stop the timer.
    /// 
    ///\sa halt()
    ///\sa start()
    ///\sa restart()
    ///\sa reset()

    void stop() 
    {
      if(_running && !--_running) {
	TimeStamp t;
	t.stamp();
	start_time=t-start_time;
      }
    }

    ///Halt (i.e stop immediately) the time counters

    ///This function stops immediately the time counters, i.e. <tt>t.halt()</tt>
    ///is a faster
    ///equivalent of the following.
    ///\code
    ///  while(t.running()) t.stop()
    ///\endcode
    ///
    ///
    ///\sa stop()
    ///\sa restart()
    ///\sa reset()

    void halt() 
    {
      if(_running) {
	_running=0;
	TimeStamp t;
	t.stamp();
	start_time=t-start_time;
      }
    }

    ///Returns the running state of the timer

    ///This function returns the number of stop() exections that is
    ///necessary to really stop the timer.
    ///For example the timer
    ///is running if and only if the return value is \c true
    ///(i.e. greater than
    ///zero).
    int running()  { return _running; }
    
    
    ///Restart the time counters

    ///This function is a shorthand for
    ///a reset() and a start() calls.
    ///
    void restart() 
    {
      reset();
      start();
    }
    
    ///@}

    ///\name Query Functions for the ellapsed time

    ///@{

    ///Gives back the ellapsed user time of the process
    double userTime() const
    {
      return operator TimeStamp().userTime();
    }
    ///Gives back the ellapsed system time of the process
    double systemTime() const
    {
      return operator TimeStamp().systemTime();
    }
    ///Gives back the ellapsed user time of the process' children
    double cUserTime() const
    {
      return operator TimeStamp().cUserTime();
    }
    ///Gives back the ellapsed user time of the process' children
    double cSystemTime() const
    {
      return operator TimeStamp().cSystemTime();
    }
    ///Gives back the ellapsed real time
    double realTime() const
    {
      return operator TimeStamp().realTime();
    }
    ///Computes the ellapsed time

    ///This conversion computes the ellapsed time, therefore you can print
    ///the ellapsed time like this.
    ///\code
    ///  Timer t;
    ///  doSomething();
    ///  std::cout << t << '\n';
    ///\endcode
    operator TimeStamp () const
    {
      TimeStamp t;
      t.stamp();
      return _running?t-start_time:start_time;
    }


    ///@}
  };

  ///Same as \ref Timer but prints a report on destruction.

  ///Same as \ref Timer but prints a report on destruction.
  ///This example shows its usage.
  ///\code
  ///  void myAlg(ListGraph &g,int n)
  ///  {
  ///    TimeReport tr("Running time of myAlg: ");
  ///    ... //Here comes the algorithm
  ///  }
  ///\endcode
  ///
  ///\sa Timer
  ///\sa NoTimeReport
  ///\todo There is no test case for this
  class TimeReport : public Timer 
  {
    std::string _title;
    std::ostream &_os;
  public:
    ///\e

    ///\param title This text will be printed before the ellapsed time.
    ///\param os The stream to print the report to.
    ///\param run Sets whether the timer should start immediately.

    TimeReport(std::string title,std::ostream &os=std::cerr,bool run=true) 
      : Timer(run), _title(title), _os(os){}
    ///\e Prints the ellapsed time on destruction.
    ~TimeReport() 
    {
      _os << _title << *this << std::endl;
    }
  };
      
  ///'Do nothing' version of \ref TimeReport

  ///\sa TimeReport
  ///
  class NoTimeReport
  {
  public:
    ///\e
    NoTimeReport(std::string,std::ostream &,bool) {}
    ///\e
    NoTimeReport(std::string,std::ostream &) {}
    ///\e
    NoTimeReport(std::string) {}
    ///\e Do nothing.
    ~NoTimeReport() {}

    operator TimeStamp () const { return TimeStamp(); }
    void reset() {}
    void start() {}
    void stop() {}
    void halt() {} 
    int running() { return 0; }
    void restart() {}
    double userTime() const { return 0; }
    double systemTime() const { return 0; }
    double cUserTime() const { return 0; }
    double cSystemTime() const { return 0; }
    double realTime() const { return 0; }
  };
      
  ///Tool to measure the running time more exactly.
  
  ///This function calls \c f several times and returns the average
  ///running time. The number of the executions will be choosen in such a way
  ///that the full real running time will be roughly between \c min_time
  ///and <tt>2*min_time</tt>.
  ///\param f the function object to be measured.
  ///\param min_time the minimum total running time.
  ///\retval num if it is not \c NULL, then the actual
  ///        number of execution of \c f will be written into <tt>*num</tt>.
  ///\retval full_time if it is not \c NULL, then the actual
  ///        total running time will be written into <tt>*full_time</tt>.
  ///\return The average running time of \c f.
  
  template<class F>
  TimeStamp runningTimeTest(F f,double min_time=10,unsigned int *num = NULL,
                            TimeStamp *full_time=NULL)
  {
    TimeStamp full;
    unsigned int total=0;
    Timer t;
    for(unsigned int tn=1;tn <= 1U<<31 && full.realTime()<=min_time; tn*=2) {
      for(;total<tn;total++) f();
      full=t;
    }
    if(num) *num=total;
    if(full_time) *full_time=full;
    return full/total;
  }
  
  /// @}  


} //namespace lemon

#endif //LEMON_TIME_MEASURE_H
