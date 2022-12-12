/*
 * String.h
 *
 *  Created on: 25 mar 2021
 *      Author: Grzegorz
 */

#ifndef KPHOST_CXSTRING_H_
#define KPHOST_CXSTRING_H_

class CxString {
private:
	char *mPStr;
	int mSize;
	int mLen;

public:
	CxString(int size);
	virtual ~CxString();
	char* p() {
		return mPStr;
	}
	int size() {
		return mSize;
	}
	int len() {
		return mLen;
	}
	char* resP() {
		return &mPStr[mLen];
	}
	int restSize() {
		return mSize - mLen;
	}
	bool isFull(){
		return (mLen==mSize);
	}
	void setLen(int L);
	void clear();
	void add(char ch);
	void add(const char *str);
	void add(CxString *src);
	void addQuota(const char *str);

	void format(const char *pFormat, ...);
	void addFormat(const char *pFormat, ...);
	void buildLen();
	void fillSpaces(int end);


};

#endif /* KPHOST_STRING_H_ */
