/*
 * TaskClass.h
 *
 *  Created on: Oct 8, 2020
 *      Author: Grzegorz
 */

#ifndef TASKCLASS_H_
#define TASKCLASS_H_

#include "cmsis_os.h"
#include "MsgStream.h"

enum {
	SIGNAL_WAKEUP = 0x00010000, //

};

class TaskClass {
	friend class TaskClassList;
private:
	osThreadDef_t mThreadDef;
	char mThreadName[0x32];
	osThreadId mThreadId;
	bool mRunning;
	bool mSuspend;
	uint32_t mAliveTick;     // ostatni czas wywołania funkcji imAlive
	uint32_t mLoopCnt;       // zliczanie wywołań funkcji imAlive w ciągu sekundy
	uint32_t mLoopCntPerSek; // ilość wywołań funkcji imAlive na sekunde
	uint32_t maxAliveBreak;  // maksymalny czas pomiędzy wywołaniami funkcji imAlive
	void every1sek();
	virtual void every1msek(){

	}


protected:
	int mDebug;
	uint32_t mAlivePeriod;
	virtual void ThreadFunc();
	void imAlive();
	bool isAlive();

	void wakeUpMe();

public:
	TaskClass(const char *name, osPriority aPriority, int stacksize);
	virtual ~TaskClass() {
	}
	void setTaskName(const char *name);
	void Suspend();
	void Resume();

	void RunThread();
	void Start();
	osThreadId getThreadId() {
		return mThreadId;
	}
	bool getRunning() {
		return mRunning;
	}
	const char *getThreadName() {
		return mThreadName;
	}
	bool isSuspended(){
		return mSuspend;
	}
};





class TaskClassList {
	friend TaskClass;
private:
	enum {
		MAX_TASK_CNT = 24,
	};
	static TaskClass *taskList[MAX_TASK_CNT];
	static TaskClass *taskListEvery1ms[MAX_TASK_CNT];  //list tasków dla których wywoływana jest funkcja every1msek()

	static int taskCnt;
	static int taskEver1msCnt;
	static int getTaskDebug(TaskClass *task);
	static int FindTask(TaskClass *aTask);
public:
	static void Add1msTask(TaskClass *task);
	static void Register(TaskClass *task);
	static void ShowList(MsgStream *strm);
	static void every1sek();
	static void every1msek();
	static TaskClass *isTasksAlive();
};





#endif /* TASKCLASS_H_ */


