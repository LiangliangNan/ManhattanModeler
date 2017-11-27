
#ifndef _COLOR_H_
#define _COLOR_H_

#include "basic_types.h"

#include <cassert>
#include <iostream>


template <class RT> 
class GenericColor
{
public:
	GenericColor(RT r=0, RT g=0, RT b=0, RT a=1.0f) ;
	RT r() const ;
	RT g() const ;
	RT b() const ;
	RT a() const ;

	void set_r(RT r) ;
	void set_g(RT g) ;
	void set_b(RT b) ;
	void set_a(RT a) ;

	RT& operator[](int i) ;
	const RT& operator[](int i) const ;

	void set(RT r, RT g, RT b, RT a=1.0f) ; 	

	// Low-level access
	const RT* data() const ;
	RT*       data() ;

private:
	RT components_[4] ;
} ;



template <class RT> inline
GenericColor<RT>::GenericColor(RT r, RT g, RT b, RT a) {
	components_[0] = r ;
	components_[1] = g ;
	components_[2] = b ;
	components_[3] = a ;
}

template <class RT> inline
RT GenericColor<RT>::r() const {
	return components_[0] ;
}

template <class RT> inline
RT GenericColor<RT>::g() const {
	return components_[1] ;
}

template <class RT> inline
RT GenericColor<RT>::b() const {
	return components_[2] ;
}

template <class RT> inline
RT GenericColor<RT>::a() const {
	return components_[3] ;
}

// Low-level access
template <class RT> inline
const RT* GenericColor<RT>::data() const { 
	return components_; 
}

template <class RT> inline
RT* GenericColor<RT>::data() {
	return components_; 
}

template <class RT> inline
void GenericColor<RT>::set(RT r, RT g, RT b, RT a=1.0f) {
	components_[0] = r ;
	components_[1] = g ;
	components_[2] = b ;
	components_[3] = a ;
}

template <class RT> inline
void GenericColor<RT>::set_r(RT r) {
	components_[0] = r ;
}

template <class RT> inline
void GenericColor<RT>::set_g(RT g) {
	components_[1] = g ;
}

template <class RT> inline
void GenericColor<RT>::set_b(RT b) {
	components_[2] = b ;
}

template <class RT> inline
void GenericColor<RT>::set_a(RT a) {
	components_[3] = a ;
}

template <class RT> inline
RT& GenericColor<RT>::operator[](int i) {
	assert(i >= 0 && i <= 3) ;
	return components_[i] ;
}

template <class RT> inline
const RT& GenericColor<RT>::operator[](int i) const {
	assert(i >= 0 && i <= 3) ;
	return components_[i] ;
}


template <class RT> inline
std::ostream& operator<<(std::ostream& output, const GenericColor<RT>& color) {
	return output << 
		color[0] << " " << color[1] << " " << color[2] << " " << color[3] ;
}

template <class RT> inline
std::istream& operator>>(std::istream& input, GenericColor<RT>& color) {
	return input >> color[0] >> color[1] >> color[2] >> color[3] ;
}



//_______________________ Colors

typedef GenericColor<Numeric::int8>    Color_int8 ;
typedef GenericColor<Numeric::uint8>   Color_uint8 ;
typedef GenericColor<Numeric::int16>   Color_int16 ;
typedef GenericColor<Numeric::uint16>  Color_uint16 ;
typedef GenericColor<Numeric::int32>   Color_int32 ;
typedef GenericColor<Numeric::uint32>  Color_uint32 ;
typedef GenericColor<Numeric::float32> Color_float32 ;
typedef GenericColor<Numeric::float64> Color_float64 ;

typedef		Color_float32	Colorf ;
typedef		Color_float64	Colord ;













namespace Global {

	Colorf		color_from_table(unsigned int index) ;
	Colorf		color_from_small_Table(unsigned int index);

	Colorf		random_color() ;
	Colorf		random_color_from_table() ;
	Colorf		color_in_range(float min_v, float max_v, float v);
}


#endif
