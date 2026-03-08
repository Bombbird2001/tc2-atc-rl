package com.bombbird.terminalcontrol2.gymnasium.staterestore

import com.bombbird.terminalcontrol2.networking.GameServer
import java.util.ArrayDeque

/**
 * Manages a bounded history of [Snapshot]s for forward-looking conflict resolution.
 * Snapshots are added in increasing temporal order; restore rewinds to an earlier state
 * and discards any snapshots that were taken after the restored point.
 *
 * @param maxSnapshots maximum number of snapshots to retain (oldest dropped when exceeded)
 */
class RLStateRestoreManager(private val maxSnapshots: Int) {

    private val snapshots = ArrayDeque<Snapshot>(maxSnapshots)

    /**
     * Creates a snapshot from the current [GameServer] state.
     * The snapshot can be restored later via [restoreSnapshot].
     */
    fun getSnapshot(gs: GameServer): Snapshot = buildSnapshot(gs)

    /**
     * Adds a snapshot to the history. Snapshots are assumed to be added in increasing temporal order.
     * Only the latest [maxSnapshots] are kept; older ones are dropped.
     */
    fun addSnapshot(snapshot: Snapshot) {
        if (maxSnapshots <= 0) return
        while (snapshots.size >= maxSnapshots) snapshots.removeFirst()
        snapshots.addLast(snapshot)
        println(snapshotCount())
    }

    /**
     * Restores the game server to the state from [stepsAgo] steps back.
     * stepsAgo = 1 restores the latest snapshot, stepsAgo = n restores the earliest available.
     * After restore, any snapshots that were added after the restored snapshot are removed.
     *
     * @param stepsAgo number of steps back (1 = latest, 2 = second latest, ...)
     * @param gs the GameServer to restore state into (modified in place)
     * @return the [Snapshot] that was restored (caller may apply bridge/reward state from it)
     * @throws IllegalArgumentException if stepsAgo is not in [1, snapshots.size]
     */
    fun restoreSnapshot(stepsAgo: Int, gs: GameServer): Snapshot {
        require(stepsAgo in 1..snapshots.size) {
            "stepsAgo must be in 1..${snapshots.size}, was $stepsAgo"
        }
        repeat(stepsAgo - 1) { snapshots.removeLast() }
        val snapshot = snapshots.last
        restoreSnapshot(snapshot, gs)
        println(snapshotCount())
        return snapshot
    }

    fun clearSnapshots() {
        snapshots.clear()
    }

    fun removeFirstSnapshot() {
        if (snapshots.isEmpty()) return
        snapshots.removeFirst()
    }

    /** Returns the number of snapshots currently held. */
    fun snapshotCount(): Int = snapshots.size
}
