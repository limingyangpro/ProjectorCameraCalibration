/*
 * hashtableInt.hpp
 *
 *  Created on: Feb 11, 2015
 *      Author: liming
 *
 *      Hash element is all unsigned long long int
 */

#ifndef LGC2LIB_HASHTABLEINT_HPP_
#define LGC2LIB_HASHTABLEINT_HPP_

#include "basictype.h"
namespace lgc2
{

class Hashtable_Boundary_exception : public exception
{
public:
	 virtual const char* what() const throw()
	  {
	    return "Hash table boundary exception";
	  }
};

/**
 * This class is the hashtable
 * Offers :
 * 1. Creation : a hash element into the hashtable
 * 2. Match : Find a correspond point for the given point
 * 3. Use 3 hashtables, first for main point = ptA, second for main point = ptB, third for mainu point ptC
 */
class HashTable
{
public:
    enum  //retrive type in the function retrieveAt
    {
    	MODEL      =  1, //! only retrieve model number
		MODELBASIS =  2, //! only retrieve model and basis
		ALLNOCOLOR =  3, //! retrive model, basis, and the point from which the index is calculated
		ALLCOLOR   =  4  //! with additonal color information for each point
    };

private:
	vector< set<unsigned long long> > metadata;     //model (X), ptA (XXX), ptB(XXX), ptC(XXX), ptD(XXX)
	int hashsize;                                   //each dimension is devided into hashsize bins
	float indexMax;
	float normalize(float x);		                //convert 2D hashkey to 1D hashindex
public:
	HashTable();
	HashTable(int hashsize_, float indexMax_);
	void setSize(int size);

	set<unsigned long long>* retrieveAt(int tableID, RDM_Point hashKey, int content);    //retrieve the stored elements, make sure that each basis contain only one record
	void insert(unsigned long long element, int tableID, RDM_Point hashKey, RDM_Point dd);
	void clearAll();       //clear the hashtable

	void printInfo(ostream &outstream);
};

}




#endif /* LGC2LIB_HASHTABLEINT_HPP_ */
