/*
 * Filters.h
 *
 *  Created on: 7 gru 2022
 *      Author: Grzegorz
 */

#ifndef FILTERS_H_
#define FILTERS_H_

class FiltrIR {
private:
	bool firstDt;
	float state;
	float mK;
public:
	FiltrIR(float k);
	void inp(float x);
	float out();
	void setK(float K) {
		mK = K;
	}
};

class FiltrFIR {
private:
	enum {
		MAX_LEN = 120, //
	};
	float tab[MAX_LEN];
	bool mOverride; // czy pamięc filtru już się przewinęła
	int mPtr;
	int mLen; // długość filtru
public:
	FiltrFIR(int len);
	void inp(float x);
	float out();
};

#endif /* FILTERS_H_ */
