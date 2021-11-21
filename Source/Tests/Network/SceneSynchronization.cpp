//
// Copyright (c) 2017-2021 the rbfx project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR rhs
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR rhsWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR rhs DEALINGS IN
// THE SOFTWARE.
//

#include "../CommonUtils.h"
#include "../NetworkUtils.h"
#include "../SceneUtils.h"

#include <Urho3D/Network/Network.h>
#include <Urho3D/Network/NetworkManager.h>
#include <Urho3D/Scene/Scene.h>

TEST_CASE("Time is synchronized between client and server")
{
    auto context = Tests::CreateNetworkSimulatorContext();
    const float frameErrorTolarance = 0.8f;

    // Prepare test parameters
    const auto retry = GENERATE(take(5, value(0)));
    const auto [minPing, maxPing] = GENERATE(
        ea::make_pair(0.1f, 0.1f),
        ea::make_pair(0.08f, 0.12f),
        ea::make_pair(0.22f, 0.28f)
    );

    unsigned seed = retry;
    CombineHash(seed, MakeHash(minPing));
    CombineHash(seed, MakeHash(maxPing));

    // Setup scenes
    auto serverScene = MakeShared<Scene>(context);
    auto clientScene = MakeShared<Scene>(context);

    Tests::NetworkSimulator sim(serverScene, seed);
    sim.AddClient(clientScene, minPing, maxPing);

    auto& serverNetworkManager = serverScene->GetNetworkManager()->AsServer();
    auto& clientNetworkManager = clientScene->GetNetworkManager()->AsClient();

    // Simulate a few millseconds, not enough for synchronization due to ping
    sim.SimulateTime(24 / 1024.0f);
    REQUIRE_FALSE(clientNetworkManager.IsSynchronized());

    // Simulate the rest of a second, should be somehow synchronized
    sim.SimulateTime(1000 / 1024.0f);

    REQUIRE(clientNetworkManager.IsSynchronized());
    REQUIRE(clientNetworkManager.GetPingInMs() == RoundToInt((maxPing + minPing) * 1000 / 2));

    const float syncError = ea::max(0.5f, (maxPing - minPing) * Tests::NetworkSimulator::FramesInSecond);
    REQUIRE(serverNetworkManager.GetCurrentFrame() == 32);
    REQUIRE(std::abs(clientNetworkManager.GetCurrentFrameDeltaRelativeTo(32)) < syncError);

    // Simulate 10 more seconds, should be precisely synchronized afterwards
    sim.SimulateTime(10.0f);
    REQUIRE(serverNetworkManager.GetCurrentFrame() == 32 * 11);
    REQUIRE(std::abs(clientNetworkManager.GetCurrentFrameDeltaRelativeTo(32 * 11)) < syncError);

    // Simulate 10 more seconds, expect time to stay synchronized
    const unsigned syncFrame1 = clientNetworkManager.GetLastSynchronizationFrame();
    sim.SimulateTime(10.0f);

    REQUIRE(serverNetworkManager.GetCurrentFrame() == 32 * 21);
    REQUIRE(std::abs(clientNetworkManager.GetCurrentFrameDeltaRelativeTo(32 * 21)) < frameErrorTolarance);
    REQUIRE(clientNetworkManager.GetLastSynchronizationFrame() == syncFrame1);

    // Wrap time to 1000000 and simulate 10 more seconds, expect time to be resynchronized
    serverNetworkManager.SetCurrentFrame(1000000);
    sim.SimulateTime(10.0f);

    REQUIRE(serverNetworkManager.GetCurrentFrame() == 1000000 + 32 * 10);
    REQUIRE(std::abs(clientNetworkManager.GetCurrentFrameDeltaRelativeTo(1000000 + 32 * 10)) < frameErrorTolarance);
    REQUIRE(clientNetworkManager.GetLastSynchronizationFrame() >= 1000000);

    // Simulate 10 more seconds, expect time to stay synchronized
    const unsigned syncFrame2 = clientNetworkManager.GetLastSynchronizationFrame();
    sim.SimulateTime(10.0f);

    REQUIRE(serverNetworkManager.GetCurrentFrame() == 1000000 + 32 * 20);
    REQUIRE(std::abs(clientNetworkManager.GetCurrentFrameDeltaRelativeTo(1000000 + 32 * 20)) < frameErrorTolarance);
    REQUIRE(clientNetworkManager.GetLastSynchronizationFrame() == syncFrame2);

    // Wrap time back to 32 and simulate 10 more seconds, expect time to be resynchronized
    serverNetworkManager.SetCurrentFrame(32);
    sim.SimulateTime(10.0f);

    REQUIRE(serverNetworkManager.GetCurrentFrame() == 32 * 11);
    REQUIRE(std::abs(clientNetworkManager.GetCurrentFrameDeltaRelativeTo(32 * 11)) < frameErrorTolarance);
    REQUIRE(clientNetworkManager.GetLastSynchronizationFrame() >= 32);

    // Simulate 10 more seconds, expect time to stay synchronized
    const unsigned syncFrame3 = clientNetworkManager.GetLastSynchronizationFrame();
    sim.SimulateTime(10.0f);

    REQUIRE(serverNetworkManager.GetCurrentFrame() == 32 * 21);
    REQUIRE(std::abs(clientNetworkManager.GetCurrentFrameDeltaRelativeTo(32 * 21)) < frameErrorTolarance);
    REQUIRE(clientNetworkManager.GetLastSynchronizationFrame() == syncFrame3);
}