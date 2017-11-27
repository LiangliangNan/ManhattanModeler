#include "ProgressBar.h"
 
#include <cassert>


Progress* Progress::instance_ = NULL ;

Progress::Progress() : 
	client_(NULL), 
	level_(0), 
	canceled_(false) 
{
}

Progress::~Progress() 
{
}

void Progress::push() 
{ 
	level_++ ; 
	if(level_ == 1) 
	{
		clear_canceled() ;
	}
}

void Progress::pop() 
{
	assert(level_ > 0) ;
	level_-- ;
}

void Progress::notify(int new_val) 
{
	if(client_ != NULL && level_ < 3) 
	{
		client_->notify_progress(new_val) ;
	}
}

Progress* Progress::instance() 
{
	if(instance_ == NULL) 
	{
		instance_ = new Progress ;
	}
	return instance_ ;
}


//_________________________________________________________


ProgressClient::~ProgressClient() {
	if (Progress::instance())
		delete Progress::instance();
}

//_________________________________________________________
 

ProgressLogger::ProgressLogger( int max_val /*= 100*/, bool quiet /*= false*/ ): max_val_(max_val), quiet_(quiet)
{
	cur_val_ = 0 ; 
	cur_percent_ = 0 ;
	Progress::instance()->push() ;
	if(!quiet_) 
	{
		Progress::instance()->notify(0) ;
	}
} 
void ProgressLogger::reset(int max_val)  
{
	max_val_ = max_val ;
	reset() ;
}

ProgressLogger::~ProgressLogger() 
{
	Progress::instance()->notify(100) ;
	Progress::instance()->pop() ;
	Progress::instance()->notify(0);
}

void ProgressLogger::next() 
{
	update() ;
	cur_val_++ ;
}

void ProgressLogger::notify(int new_val) 
{
	cur_val_ = new_val ;
	update() ;
}

template <class T> 
inline T ogf_max(T x1, T x2) 
{
	return x1 > x2 ? x1 : x2;
}

template <class T> 
inline T ogf_min(T x1, T x2) {
	return x1 < x2 ? x1 : x2;
}

void ProgressLogger::update() 
{
	int percent = cur_val_ * 100 / ogf_max(1, max_val_-1) ;
	if(percent != cur_percent_) 
	{
		cur_percent_ = percent ;
		if(!quiet_) 
		{
			Progress::instance()->notify(ogf_min(cur_percent_, 100)) ;
		}
	}
}

