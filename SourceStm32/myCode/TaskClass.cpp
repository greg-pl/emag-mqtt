/*
 * TaskClass.cpp
 *
 *  Created on: Oct 8, 2020
 *      Author: Grzegorz
 */

#include "TaskClass.h"

#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "main.h"
#include "TaskClass.h"

void ThreadEntryFunc(void const *argument) {
	TaskClass *task = (TaskClass*) (unsigned int) argument;
	task->RunThread();
}

TaskClass::TaskClass(const char *name, osPriority aPriority, int stacksize) {
	strncpy(mThreadName, name, sizeof(mThreadName));
	memset(&mThreadDef, 0, sizeof(mThreadDef));
	mThreadDef.name = mThreadName;
	mThreadDef.stacksize = stacksize;
	mThreadDef.tpriority = aPriority;
	mThreadDef.instances = 0;
	mThreadDef.pthread = ThreadEntryFunc;
	mRunning = false;
	mSuspend = false;
	mThreadId = NULL;
	mDebug = 0;
	mAlivePeriod = 2000;
	mAliveTick = 0;
	mLoopCnt = 0;
	mLoopCntPerSek = 0;
	maxAliveBreak = 0;
	TaskClassList::Register(this);
}

//musi być wywołane przed Start
void TaskClass::setTaskName(const char *name) {
	strncpy(mThreadName, name, sizeof(mThreadName));
}

void TaskClass::Start() {
	mThreadId = osThreadCreate(&mThreadDef, (void*) this);
	printf("Task %s, id=%d\r\n", mThreadName, (int) uxTaskGetTaskNumber(mThreadId));
	if (mThreadId == NULL)
		printf("osThreadCreate ERROR\r\n");
}

void TaskClass::RunThread() {
	mRunning = true;
	ThreadFunc();
	printf("Task %s, EIXT\r\n", mThreadName);
	vTaskDelete( NULL);
}

void TaskClass::ThreadFunc() {
	while (1) {
		osDelay(2000);
		printf("Task %s\r\n", mThreadName);
	}
}

void TaskClass::every1sek() {
	mLoopCntPerSek = mLoopCnt;
	mLoopCnt = 0;
}

void TaskClass::imAlive() {
	uint32_t tt = HAL_GetTick();
	uint32_t dt = tt - mAliveTick;
	if (dt > maxAliveBreak)
		maxAliveBreak = dt;

	mAliveTick = tt;
	mLoopCnt++;
}

bool TaskClass::isAlive() {
	return (HAL_GetTick() - mAliveTick < mAlivePeriod);
}

void TaskClass::Suspend() {
	mSuspend = true;
	osThreadSuspend(mThreadId);
	mSuspend = false;
}

void TaskClass::Resume() {
	osThreadResume(mThreadId);
}

void TaskClass::wakeUpMe() {
	if (mThreadId != NULL) {
		osSignalSet(mThreadId, SIGNAL_WAKEUP);
	}
}

//--------------------------------------------------------------------------
// TaskClassList
//--------------------------------------------------------------------------
TaskClass *TaskClassList::taskList[MAX_TASK_CNT];
TaskClass *TaskClassList::taskListEvery1ms[MAX_TASK_CNT];

int TaskClassList::taskCnt;
int TaskClassList::taskEver1msCnt;

void TaskClassList::Register(TaskClass *task) {
	if (taskCnt < MAX_TASK_CNT) {
		taskList[taskCnt++] = task;
	} else {
		printf("MAX_TASK_CNT too small\r\n");
	}
}

void TaskClassList::every1sek() {
	for (int i = 0; i < taskCnt; i++)
		taskList[i]->every1sek();
}

void TaskClassList::every1msek() {
	for (int i = 0; i < taskEver1msCnt; i++) {
		taskListEvery1ms[i]->every1msek();
	}
}

void TaskClassList::Add1msTask(TaskClass *task) {
	taskListEvery1ms[taskEver1msCnt] = task;
	taskEver1msCnt++;
}

int printTaskInf(char *buf, int max, TaskStatus_t *tsk) {
	return snprintf(buf, max, "%3u|%16s|%p|%5u|%2u|%2u|%5u|",	//
			(int) tsk->xTaskNumber, //
			tsk->pcTaskName,	//
			tsk->pxStackBase, //
			tsk->usStackHighWaterMark, //
			(int) tsk->uxBasePriority, //
			(int) tsk->uxCurrentPriority, //
			(int) tsk->ulRunTimeCounter);
}

void TaskClassList::ShowList(OutStream *strm) {
	int tCnt = uxTaskGetNumberOfTasks();

	TaskStatus_t *aTaskBuf = (TaskStatus_t*) malloc(tCnt * sizeof(TaskStatus_t));
	bool *flagsBuf = (bool*) malloc(tCnt * sizeof(bool));

	if ((aTaskBuf != NULL) && (flagsBuf != NULL)) {
		tCnt = uxTaskGetSystemState(aTaskBuf, tCnt, NULL);
		memset(flagsBuf, 0, tCnt * sizeof(bool));
		if (strm->oOpen(colWHITE)) {
			strm->oMsg("lp|name            |r|loop/s|max.br|Tnr|OsName          |StackAdr  |StPos|bP|cP|Time |");
			strm->oMsg("--+----------------+-+------+------+---+----------------+----------+-----+--+--+-----+");
			char buf[120];
			for (int i = 0; i < taskCnt; i++) {
				char ch = '.';
				if (taskList[i]->mRunning)
					ch = '+';
				if (taskList[i]->mSuspend)
					ch = 'S';
				int n = snprintf(buf, sizeof(buf), "%2u|%16s|%c|%6u|%6u|", i, taskList[i]->getThreadName(), ch, (int) taskList[i]->mLoopCntPerSek, (int) taskList[i]->maxAliveBreak);
				taskList[i]->maxAliveBreak = 0;
				int idx = -1;
				for (int j = 0; j < tCnt; j++) {
					if (aTaskBuf[j].xHandle == taskList[i]->mThreadId) {
						idx = j;
						break;
					}
				}
				if (idx >= 0) {
					flagsBuf[idx] = true;
					n += printTaskInf(&buf[n], sizeof(buf) - n, &aTaskBuf[idx]);

				} else {
					n += snprintf(&buf[n], sizeof(buf) - n, "   |");
				}
				strm->oMsg(buf);
			}
			for (int j = 0; j < tCnt; j++) {
				if (!flagsBuf[j]) {
					int n = snprintf(buf, sizeof(buf), "%2u|                | |      |      |", taskCnt + j);
					n += printTaskInf(&buf[n], sizeof(buf) - n, &aTaskBuf[j]);
					strm->oMsg(buf);
				}
			}

			strm->oMsg("--+----------------+-+------+------+---+----------------+----------+-----+--+--+-----+\r\n");
			strm->oClose();
		}

	}
	free(flagsBuf);
	free(aTaskBuf);
}

int TaskClassList::FindTask(TaskClass *aTask) {
	for (int i = 0; i < taskCnt; i++) {
		if (taskList[i] == aTask) {
			return i;
		}
	}
	return -1;
}

TaskClass* TaskClassList::isTasksAlive() {
	for (int i = 0; i < taskCnt; i++) {
		if (!taskList[i]->isAlive()) {
			return taskList[i];
		}
	}
	return NULL;
}
