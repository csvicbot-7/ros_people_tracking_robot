#ifndef ROSSHAREDMEMORY_H
#define ROSSHAREDMEMORY_H

#include <string>		//string
#include <cstring>		//memory cpy/set
#include <sys/mman.h>	//shared memory
#include <sys/stat.h>	//PROT_* mode constants
#include <fcntl.h>		//O_* oflag constants
#include <semaphore.h>	//semaphore
#include <stdio.h>		//perror
#include <unistd.h>		//ftruncate

#define VersionFrameMax 10

using namespace std;

template <typename TQtToRos, typename TRosToQt>
class CRosSharedMemory
{
public:
	/*!
	 * \brief	Function to create SlamSharedMemory object
	 * \param	bResetSemaphore: reset named semaphore that locks acces to shared memory,
	 *			first aplication that creates shared memory should reset the semaphore just
	 *			in case that it already exists and its locked from a previous execution.
	 *			False by default
	 * \param	sName: name of the shared memory device
	 * \param	mode: mode to acces to shared memory dedvice: read-write O_RDWR, or read only O_RDONLY.
	 *			O_RDWR by default
	 */
	CRosSharedMemory(const string sName, bool bResetSemaphore = false, int mode = O_RDWR) {
		m_sName = sName;
		m_sharedId = -1;
		m_sharedPtr = nullptr;
		m_semaphoreId = nullptr;
		m_versionQtLast = 0;
		m_versionRosLast = 0;
		m_countQt = 0;
		m_countRos = 0;
		m_nSize = sizeof(CRosSharedMemory::stShared);

		//Semaphore open
		string semName = sName + "_sem";
		if (bResetSemaphore)
		{
			sem_unlink(semName.c_str());
		}
		m_semaphoreId = sem_open(semName.c_str(), O_CREAT, 0644, 1);
		if (m_semaphoreId == SEM_FAILED)
		{
			perror("sem_open");
		}

		//Shared memory open
		m_sharedId = shm_open(m_sName.c_str(), O_CREAT | mode, S_IRWXU | S_IRWXG);
		if (m_sharedId < 0)
		{
			perror("shm_open");
		}
		else
		{
			Truncate(static_cast<long>(m_nSize));
			Attach();
		}
	}

	~CRosSharedMemory() {
		//Clear();
	}

	/*!
	 * \brief	Function to set new values in shared memory from Qt to ROS
	 * \param	sSharedQtToRos: data struct to be stored in shared memory space
	 * \return	true if saved successfully, false in case that we cannot get acces to
	 *			shared memory space (semaphore time out)
	 */
	bool setData(TQtToRos sSharedQtToRos) {
		if (LockTimed())
		{
			m_sharedPtr->versionQt = m_sharedPtr->versionQt + 1;
			memcpy(&m_sharedPtr->sQtToRos, &sSharedQtToRos, sizeof(TQtToRos));
			UnLock();
			return true;
		}
		return false;
	}

	/*!
	 * \brief	Function to set new values in shared memory from ROS to Qt
	 * \param	sSharedRosToQt: data struct to be stored in shared memory space
	 * \return	true if saved successfully, false in case that we cannot get acces to
	 *			shared memory space (semaphore time out)
	 */
	bool setData(TRosToQt sSharedRosToQt) {
		if (LockTimed())
		{
			m_sharedPtr->versionRos = m_sharedPtr->versionRos + 1;
			memcpy(&m_sharedPtr->sRosToQt, &sSharedRosToQt, sizeof(TRosToQt));
			UnLock();
			return true;
		}
		return false;
	}

	/*!
	 * \brief	Function to read values from shared memory from Qt to ROS
	 * \param	sSharedQtToRos: data struct where shared memory content will be read
	 * \return	true if get successfully, false in case that we cannot get acces to
	 *			shared memory space (semaphore time out) or
	 *			readed data is too old (version counter greater than frame limit)
	 */
	bool getData(TQtToRos &sSharedQtToRos) {
		bool bRet = false;
		unsigned char versionQt = 0;

		if (LockTimed())
		{
			versionQt = m_sharedPtr->versionQt;
			memcpy(&sSharedQtToRos, &m_sharedPtr->sQtToRos, sizeof(TQtToRos));
			UnLock();
			bRet = true;
		}

		if (bRet)
		{
			bRet = validateVersion(m_versionQtLast, versionQt, m_countQt);
		}
		return bRet;
	}

	/*!
	 * \brief	Function to read values from shared memory from ROS to Qt
	 * \param	sSharedRosToQt: data struct where shared memory content will be read
	 * \return	true if get successfully, false in case that we cannot get acces to
	 *			shared memory space (semaphore time out) or
	 *			readed data is too old (version counter greater than frame limit)
	 */
	bool getData(TRosToQt &sSharedRosToQt) {
		bool bRet = false;
		unsigned char versionRos = 0;

		if (LockTimed())
		{
			versionRos = m_sharedPtr->versionRos;
			memcpy(&sSharedRosToQt, &m_sharedPtr->sRosToQt, sizeof(TRosToQt));
			UnLock();
			bRet = true;
		}

		if (bRet)
		{
			bRet = validateVersion(m_versionRosLast, versionRos, m_countRos);
		}
		return bRet;
	}

	unsigned char getVersionQt() { return m_versionQtLast; }
	unsigned char getVersionRos() { return m_versionRosLast; }

	void resetCountQt() { m_countQt = 0; }
	void resetCountRos() { m_countRos = 0; }

private:
	bool Truncate(long size) {
		//adjusting mapped file size (make room for the whole segment to map) -- ftruncate()
		int nRet = ftruncate(m_sharedId, size);

		return nRet == 0;
	}
	bool Attach(int mode = PROT_READ | PROT_WRITE) {
		//requesting the shared segment -- mmap()
		m_sharedPtr = static_cast<stShared*>(mmap(nullptr, m_nSize, mode, MAP_SHARED, m_sharedId, 0));
		return m_sharedPtr != nullptr;
	}
	bool Detach() {
		int nRet = munmap(m_sharedPtr, m_nSize);
		return nRet == 0;
	}
	bool Lock() {
		int nRet = sem_wait(m_semaphoreId);
		return nRet == 0;
	}
	bool LockTimed() {
		timespec t;
		t.tv_sec = 0;
		t.tv_nsec = 200000000; //200ms
		//t.tv_nsec = 500000000; //500ms
		int nRet = sem_timedwait(m_semaphoreId, &t);
		return nRet == 0;
	}
	bool UnLock() {
		int nRet = sem_post(m_semaphoreId);
		return nRet == 0;
	}
	void Clear() {
		if (m_sharedId != -1)
		{
			if (Detach() != true)
			{
				perror("munmap");
			}
			if (shm_unlink(m_sName.c_str()) < 0)
			{
				perror("shm_unlink");
			}
		}
		//Semaphore unlink: Remove a named semaphore  from the system.
		if (m_semaphoreId != nullptr)
		{
			//Semaphore Close: Close a named semaphore
			if (sem_close(m_semaphoreId) < 0)
			{
				perror("sem_close");
			}
			string semName = m_sName + "_sem";
			//Semaphore unlink: Remove a named semaphore  from the system.
			if (sem_unlink(semName.c_str()) < 0)
			{
				perror("sem_unlink");
			}
		}
	}
	bool validateVersion(unsigned char &versionRead, unsigned char versionWrite, unsigned char &count) {
		if (versionRead == versionWrite) {
			if (count > VersionFrameMax) {
				return false;
			}
			count++;
		} else {
			versionRead = versionWrite;
			count = 0;
		}
		return true;
	}

private:
	//shared memory content
	struct stShared {
		TQtToRos sQtToRos;
		TRosToQt sRosToQt;
		unsigned char versionQt;
		unsigned char versionRos;
	};

	string			m_sName;
	size_t			m_nSize;
	int				m_sharedId;
	stShared*		m_sharedPtr;
	sem_t*			m_semaphoreId;

	unsigned char	m_versionQtLast;
	unsigned char	m_versionRosLast;
	unsigned char	m_countQt;
	unsigned char	m_countRos;
};

#endif // ROSSHAREDMEMORY_H
