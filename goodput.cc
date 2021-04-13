/*  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Italo Cunha <cunha@dcc.ufmg.br>, 2019
 */

#include <cassert>
#include <chrono>
#include <cmath>

#define MICROS_IN_SEC 1000000

enum class ModelResult {
    OK,
    ERROR_MINRTT_IS_ZERO,
    ERROR_INIT_CWND_SLOWER_THAN_1BPMS,
    ERROR_TRANSFER_FASTER_THAN_MODEL,
};

struct ModelRate {
    uint64_t bytesPerSec;
    uint64_t rttsInSlowStart;
    uint64_t projectedCwndPkts;
    uint64_t lastFullCwndPkts;
    enum ModelResult status;
};

ModelRate Gpeak(uint64_t totalBytes,
                uint64_t initCwndPkts,
                uint64_t mssBytes,
                std::chrono::microseconds minRtt) {
    if (minRtt.count() == 0) {
        return {0, 0, 0, 0, ModelResult::ERROR_MINRTT_IS_ZERO};
    }
    const int64_t xferPkts = std::ceil(double(totalBytes) / mssBytes);

    // The inverse of the sum of a geometric series gives the number of
    // RTTs before we finish transfering, assuming we never exit slow
    // start. We subtract 1 to get the number of RTTs before the last
    // one (the last RTT is special as it may not use its full cwnd).
    int64_t rttsToLast = std::ceil(std::log2(double(xferPkts) / initCwndPkts + 1)) - 1;
    assert(rttsToLast >= 0);

    int64_t lastFullCwndPkts = 0;
    if (rttsToLast > 0) {
        lastFullCwndPkts = (1 << (rttsToLast - 1)) * initCwndPkts;
    }

    // The number of packets sent prior to the last congestion window:
    int64_t ssPkts = ((1 << rttsToLast) - 1) * initCwndPkts;
    assert(ssPkts <= xferPkts);
    int64_t lastRttPkts = xferPkts - ssPkts;

    // The maximum rate we can test for is given by which RTT among the
    // last two transfers the most bytes.
    int64_t maxRttBytes = std::fmax(lastFullCwndPkts, lastRttPkts);

    int64_t GpeakBps = maxRttBytes * mssBytes * MICROS_IN_SEC / minRtt.count();
    int64_t projectedCwndPkts = initCwndPkts + xferPkts;
    int64_t lastFullCwndPktsMax = std::fmax(initCwndPkts, lastFullCwndPkts);

    return {uint64_t(GpeakBps), uint64_t(rttsToLast), uint64_t(projectedCwndPkts),
            uint16_t(lastFullCwndPktsMax), ModelResult::OK};
}

ModelRate GoodputBps(uint64_t totalBytes,
                     uint64_t initCwndPkts,
                     uint64_t mssBytes,
                     std::chrono::microseconds minRtt,
                     std::chrono::microseconds totalTime) {
    if (minRtt.count() == 0) {
        return {0, 0, 0, 0, ModelResult::ERROR_MINRTT_IS_ZERO};
    }
    const int64_t xferPkts = std::ceil(double(totalBytes) / mssBytes);

    int64_t rtts = 0;
    int64_t cumulativePkts = 0;
    int64_t cwnd = initCwndPkts;
    std::chrono::microseconds modelTime;
    // O(log2(xferPkts/initCwndPkts)) iterations
    while (xferPkts > cumulativePkts) {
        // Check if we can transmit at the next cwnd's rate
        int64_t tputBps = cwnd * mssBytes * MICROS_IN_SEC / minRtt.count();
        if (tputBps == 0) {
            return {0, 0, 0, 0, ModelResult::ERROR_INIT_CWND_SLOWER_THAN_1BPMS};
        }

        // We need to account for the transmission time of one
        // full-sized packet for each RTT while the cwnd is growing. We
        // optimize this by simply adding `rtts` packets to
        // xmissionTime.
        auto ssXmissionTime = std::chrono::microseconds(rtts * mssBytes * MICROS_IN_SEC / tputBps);

        auto xmissionTime = std::chrono::microseconds((totalBytes - cumulativePkts * mssBytes)
                * MICROS_IN_SEC / tputBps);
        modelTime = (minRtt * (rtts + 1)) + xmissionTime + ssXmissionTime;

        if (totalTime >= modelTime) {
            // Cannot transfer at the next cwnd rate. Found the number of RTTs in slow start.
            break;
        }
        rtts++;
        cumulativePkts += cwnd;
        cwnd <<= 1;
    }

    if (xferPkts <= cumulativePkts) {
        // The connection is very fast and seems to have finished without leaving slow start.
        assert(totalTime < modelTime);
        // We backtrack one cwnd to undo the increment at the end of
        // each iteration and retrieve the amount of RTTs when a full
        // cwnd was transmitted while in slow start. The connection may
        // be faster than our model predicts; we check for these cases
        // and return ModelResult::ERROR_TRANSFER_FASTER_THAN_MODEL
        // below.
        cwnd >>= 1;
        cumulativePkts -= cwnd;
        rtts--;
    }

    std::chrono::microseconds remainingTime = totalTime - (minRtt * (rtts + 1));
    if (remainingTime.count() <= 0) {
        // It appears the connection is faster than our model predicts, e.g., because the cwnd
        // grew *faster* than exponentially.
        return {0, 0, 0, 0, ModelResult::ERROR_TRANSFER_FASTER_THAN_MODEL};
    }

    // We add `rtts` packets to remainingPkts as we have not included
    // the transmission time of these in remainingTimeSec above as the
    // rate is yet unknown. (remainingTime is larger than in the
    // model, so we increase remainingBytes proportionately.) This is
    // the added "drift" delay due to transmission times between
    // consecutive cwnd transmissions during slow start.
    int64_t remainingPkts = xferPkts - cumulativePkts + rtts;
    assert(remainingPkts > 0);
    int64_t remainingBytes = remainingPkts * mssBytes;
    int64_t achievedBps = remainingBytes * MICROS_IN_SEC / remainingTime.count();

    int64_t projectedCwndPkts = achievedBps * minRtt.count() / MICROS_IN_SEC / mssBytes;

    return {uint32_t(achievedBps), uint32_t(rtts), uint64_t(projectedCwndPkts),
            uint64_t(cwnd), ModelResult::OK};
}