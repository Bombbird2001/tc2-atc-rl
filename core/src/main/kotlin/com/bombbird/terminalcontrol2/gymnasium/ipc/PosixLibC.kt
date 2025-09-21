package com.bombbird.terminalcontrol2.gymnasium.ipc

import com.sun.jna.*

class Timespec(val sec: Long, val nsec: Long): Structure() {
    @JvmField var tv_sec: Long = sec
    @JvmField var tv_nsec: Long = nsec
    override fun getFieldOrder() = listOf("tv_sec", "tv_nsec")
}

object PosixLibC: Library {
    init {
        Native.register("c")
    }

    external fun shm_open(name: String, oflag: Int, mode: Int): Int
    external fun mmap(addr: Pointer?, length: Long, prot: Int, flags: Int, fd: Int, offset: Long): Pointer

    external fun sem_open(name: String, oflag: Int): Pointer
    external fun sem_wait(sem: Pointer): Int
    external fun sem_trywait(sem: Pointer): Int
    external fun sem_timedwait(sem: Pointer, absTimeout: Timespec): Int
    external fun sem_post(sem: Pointer): Int
}

object MacOSLibC: Library {
    init {
        Native.register("c")
    }

    external fun shm_open(name: String, oflag: Int, mode: Int): Int
    external fun mmap(addr: Pointer?, length: Long, prot: Int, flags: Int, fd: Int, offset: Long): Pointer

    external fun sem_open(name: String, oflag: Int): Pointer
    external fun sem_wait(sem: Pointer): Int
    external fun sem_trywait(sem: Pointer): Int
    external fun sem_post(sem: Pointer): Int
}
