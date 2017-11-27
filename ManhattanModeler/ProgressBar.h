#pragma once 

#include <string>


class ProgressClient ;
 
class Progress 
{
public:
	Progress() ;
	virtual ~Progress() ;
	virtual void notify(int new_val) ;
	static Progress* instance() ;
	void set_client(ProgressClient* c) { client_ = c ; }
	void push() ;
	void pop() ;
	void cancel()            { canceled_ = true ;  }
	void clear_canceled()    { canceled_ = false ; }
	bool is_canceled() const { return canceled_ ;  }
private:
	static Progress* instance_ ;
	ProgressClient* client_ ;
	int level_ ;
	bool canceled_ ;
} ;

//_________________________________________________________
 
class ProgressClient 
{
public:
	virtual void notify_progress(int new_val) = 0;
	virtual ~ProgressClient() ;
} ;

//_________________________________________________________

class ProgressLogger 
{
public:
	ProgressLogger(int max_val = 100, bool quiet = false) ;
	virtual ~ProgressLogger() ;
	virtual void notify(int new_val) ;
	virtual void next() ;
	bool is_canceled() const 
	{
		return Progress::instance()->is_canceled() ;
	}
	void reset() { notify(0) ; }
	void reset(int max_val) ;

protected:
	virtual void update() ;

private:
	int max_val_ ;
	int cur_val_ ;
	int cur_percent_ ;
	bool quiet_ ;
} ; 

