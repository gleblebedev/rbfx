//
// Copyright (c) 2008-2020 the Urho3D project.
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
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Core/ProcessUtils.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/UI/Text.h>
#include <Urho3D/UI/UI.h>

#include "ParticleGraph.h"

#include "Urho3D/Graphics/DebugRenderer.h"
#include "Urho3D/Graphics/Model.h"
#include "Urho3D/Particles/ParticleGraphEffect.h"
#include "Urho3D/Particles/ParticleGraphEmitter.h"
#include "Urho3D/Physics/CollisionShape.h"
#include "Urho3D/Physics/PhysicsWorld.h"
#include <Urho3D/Physics/RigidBody.h>

#include <Urho3D/Graphics/StaticModel.h>
#include <Urho3D/Graphics/Zone.h>
#include <Urho3D/Graphics/Octree.h>

#include <Urho3D/DebugNew.h>
ParticleGraph::ParticleGraph(Context* context)
    : Sample(context)
    , drawDebug_(false)
{
}

void ParticleGraph::Start()
{
    // Execute base class startup
    Sample::Start();

    // Create the scene content
    CreateScene();

    // Create the UI content
    CreateInstructions();

    // Setup the viewport for displaying the scene
    SetupViewport();

    // Hook up to the frame update and render post-update events
    SubscribeToEvents();

    // Set the mouse mode to use in the sample
    Sample::InitMouseMode(MM_RELATIVE);
}

void ParticleGraph::CreateScene()
{
    auto* cache = GetSubsystem<ResourceCache>();

    scene_ = new Scene(context_);

    // Create octree, use default volume (-1000, -1000, -1000) to (1000, 1000, 1000)
    // Also create a DebugRenderer component so that we can draw debug geometry
    scene_->CreateComponent<Octree>();
    scene_->CreateComponent<DebugRenderer>();
    scene_->CreateComponent<PhysicsWorld>();
    
    // Create a Zone component for ambient lighting & fog control
    Node* zoneNode = scene_->CreateChild("Zone");
    auto* zone = zoneNode->CreateComponent<Zone>();
    zone->SetBoundingBox(BoundingBox(-1000.0f, 1000.0f));
    zone->SetAmbientColor(Color(0.1f, 0.1f, 0.1f));
    zone->SetFogStart(100.0f);
    zone->SetFogEnd(300.0f);

    // Create a directional light without shadows
    Node* lightNode = scene_->CreateChild("DirectionalLight");
    lightNode->SetDirection(Vector3(0.5f, -1.0f, 0.5f));
    auto* light = lightNode->CreateComponent<Light>();
    light->SetLightType(LIGHT_DIRECTIONAL);
    light->SetColor(Color(0.2f, 0.2f, 0.2f));
    light->SetSpecularIntensity(1.0f);

    // Create a "floor" consisting of several tiles
    for (int y = -5; y <= 5; ++y)
    {
        for (int x = -5; x <= 5; ++x)
        {
            Node* floorNode = scene_->CreateChild("FloorTile");
            floorNode->SetPosition(Vector3(x * 20.5f, -0.5f, y * 20.5f));
            floorNode->SetScale(Vector3(20.0f, 1.0f, 20.f));
            auto* floorObject = floorNode->CreateComponent<StaticModel>();
            floorObject->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
            floorObject->SetMaterial(cache->GetResource<Material>("Materials/Stone.xml"));
            auto* floorCollider = floorNode->CreateComponent<CollisionShape>();
            floorCollider->SetBox(Vector3::ONE);
            auto* floorRigidBody = floorNode->CreateComponent<RigidBody>();
            floorRigidBody->SetCollisionLayer(2);
        }
    }

    // Create groups of mushrooms, which act as shadow casters
    const unsigned NUM_MUSHROOMGROUPS = 25;
    const unsigned NUM_MUSHROOMS = 25;

    for (unsigned i = 0; i < NUM_MUSHROOMGROUPS; ++i)
    {
        // First create a scene node for the group. The individual mushrooms nodes will be created as children
        Node* groupNode = scene_->CreateChild("MushroomGroup");
        groupNode->SetPosition(Vector3(Random(190.0f) - 95.0f, 0.0f, Random(190.0f) - 95.0f));

        for (unsigned j = 0; j < NUM_MUSHROOMS; ++j)
        {
            Node* mushroomNode = groupNode->CreateChild("Mushroom");
            mushroomNode->SetPosition(Vector3(Random(25.0f) - 12.5f, 0.0f, Random(25.0f) - 12.5f));
            mushroomNode->SetRotation(Quaternion(0.0f, Random() * 360.0f, 0.0f));
            mushroomNode->SetScale(1.0f + Random() * 4.0f);
            auto* mushroomObject = mushroomNode->CreateComponent<StaticModel>();
            mushroomObject->SetModel(cache->GetResource<Model>("Models/Mushroom.mdl"));
            mushroomObject->SetMaterial(cache->GetResource<Material>("Materials/Mushroom.xml"));
            mushroomObject->SetCastShadows(true);
            auto* body = mushroomNode->CreateComponent<RigidBody>();
            body->SetCollisionLayer(2);
            auto* shape = mushroomNode->CreateComponent<CollisionShape>();
            shape->SetTriangleMesh(mushroomObject->GetModel(), 0);

        }
    }

    // Create billboard sets (floating smoke)
    const unsigned NUM_BILLBOARDNODES = 25;

    for (unsigned i = 0; i < NUM_BILLBOARDNODES; ++i)
    {
        Node* smokeNode = scene_->CreateChild("Smoke");
        smokeNode->SetPosition(Vector3(Random(200.0f) - 100.0f, Random(20.0f) + 10.0f, Random(200.0f) - 100.0f));

        auto* billboardObject = smokeNode->CreateComponent<ParticleGraphEmitter>();
        billboardObject->SetEffect(cache->GetResource<ParticleGraphEffect>("Particle/SnowExplosionGraph.xml"));
        billboardObject->EmitNewParticle(0);
    }

    // Create shadow casting spotlights
    const unsigned NUM_LIGHTS = 9;

    for (unsigned i = 0; i < NUM_LIGHTS; ++i)
    {
        Node* lightNode = scene_->CreateChild("SpotLight");
        auto* light = lightNode->CreateComponent<Light>();

        float angle = 0.0f;

        Vector3 position((i % 3) * 60.0f - 60.0f, 45.0f, (i / 3.f) * 60.0f - 60.0f);
        Color color(((i + 1) & 1u) * 0.5f + 0.5f, (((i + 1) >> 1u) & 1u) * 0.5f + 0.5f,
                    (((i + 1) >> 2u) & 1u) * 0.5f + 0.5f);

        lightNode->SetPosition(position);
        lightNode->SetDirection(Vector3(Sin(angle), -1.5f, Cos(angle)));

        light->SetLightType(LIGHT_SPOT);
        light->SetRange(90.0f);
        light->SetRampTexture(cache->GetResource<Texture2D>("Textures/RampExtreme.png"));
        light->SetFov(45.0f);
        light->SetColor(color);
        light->SetSpecularIntensity(1.0f);
        light->SetCastShadows(true);
        light->SetShadowBias(BiasParameters(0.00002f, 0.0f));

        // Configure shadow fading for the lights. When they are far away enough, the lights eventually become
        // unshadowed for better GPU performance. Note that we could also set the maximum distance for each object to
        // cast shadows
        light->SetShadowFadeDistance(100.0f); // Fade start distance
        light->SetShadowDistance(125.0f);     // Fade end distance, shadows are disabled
        // Set half resolution for the shadow maps for increased performance
        light->SetShadowResolution(0.5f);
        // The spot lights will not have anything near them, so move the near plane of the shadow camera farther
        // for better shadow depth resolution
        light->SetShadowNearFarRatio(0.01f);
    }

    // Create the camera. Limit far clip distance to match the fog
    cameraNode_ = scene_->CreateChild("Camera");
    auto* camera = cameraNode_->CreateComponent<Camera>();
    camera->SetFarClip(300.0f);

    // Set an initial position for the camera scene node above the plane
    cameraNode_->SetPosition(Vector3(0.0f, 5.0f, 0.0f));
}

void ParticleGraph::CreateInstructions()
{
    auto* cache = GetSubsystem<ResourceCache>();
    auto* ui = GetSubsystem<UI>();

    // Construct new Text object, set string to display and font to use
    auto* instructionText = ui->GetRoot()->CreateChild<Text>();
    instructionText->SetText("Use WASD keys and mouse/touch to move\n"
                             "Space to toggle debug geometry");
    instructionText->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 15);
    // The text has multiple rows. Center them in relation to each other
    instructionText->SetTextAlignment(HA_CENTER);

    // Position the text relative to the screen center
    instructionText->SetHorizontalAlignment(HA_CENTER);
    instructionText->SetVerticalAlignment(VA_CENTER);
    instructionText->SetPosition(0, ui->GetRoot()->GetHeight() / 4);
}

void ParticleGraph::SetupViewport()
{
    auto* renderer = GetSubsystem<Renderer>();

    // Set up a viewport to the Renderer subsystem so that the 3D scene can be seen
    SharedPtr<Viewport> viewport(new Viewport(context_, scene_, cameraNode_->GetComponent<Camera>()));
    renderer->SetViewport(0, viewport);
}

void ParticleGraph::SubscribeToEvents()
{
    // Subscribe HandleUpdate() function for processing update events
    SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(ParticleGraph, HandleUpdate));

    // Subscribe HandlePostRenderUpdate() function for processing the post-render update event, during which we request
    // debug geometry
    SubscribeToEvent(E_POSTRENDERUPDATE, URHO3D_HANDLER(ParticleGraph, HandlePostRenderUpdate));
}

void ParticleGraph::MoveCamera(float timeStep)
{
    // Do not move if the UI has a focused element (the console)
    if (GetSubsystem<UI>()->GetFocusElement())
        return;

    auto* input = GetSubsystem<Input>();

    // Movement speed as world units per second
    const float MOVE_SPEED = 20.0f;
    // Mouse sensitivity as degrees per pixel
    const float MOUSE_SENSITIVITY = 0.1f;

    // Use this frame's mouse motion to adjust camera node yaw and pitch. Clamp the pitch between -90 and 90 degrees
    IntVector2 mouseMove = input->GetMouseMove();
    yaw_ += MOUSE_SENSITIVITY * mouseMove.x_;
    pitch_ += MOUSE_SENSITIVITY * mouseMove.y_;
    pitch_ = Clamp(pitch_, -90.0f, 90.0f);

    // Construct new orientation for the camera scene node from yaw and pitch. Roll is fixed to zero
    cameraNode_->SetRotation(Quaternion(pitch_, yaw_, 0.0f));

    // Read WASD keys and move the camera scene node to the corresponding direction if they are pressed
    if (input->GetKeyDown(KEY_W))
        cameraNode_->Translate(Vector3::FORWARD * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_S))
        cameraNode_->Translate(Vector3::BACK * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_A))
        cameraNode_->Translate(Vector3::LEFT * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_D))
        cameraNode_->Translate(Vector3::RIGHT * MOVE_SPEED * timeStep);

    // Toggle debug geometry with space
    if (input->GetKeyPress(KEY_SPACE))
        drawDebug_ = !drawDebug_;
}

void ParticleGraph::AnimateScene(float timeStep)
{
    // Get the light and billboard scene nodes
    ea::vector<Node*> lightNodes;
    scene_->GetChildrenWithComponent<Light>(lightNodes);

    const float LIGHT_ROTATION_SPEED = 20.0f;
    const float BILLBOARD_ROTATION_SPEED = 50.0f;

    // Rotate the lights around the world Y-axis
    for (unsigned i = 0; i < lightNodes.size(); ++i)
        lightNodes[i]->Rotate(Quaternion(0.0f, LIGHT_ROTATION_SPEED * timeStep, 0.0f), TS_WORLD);
}

void ParticleGraph::HandleUpdate(StringHash eventType, VariantMap& eventData)
{
    using namespace Update;

    // Take the frame time step, which is stored as a float
    float timeStep = eventData[P_TIMESTEP].GetFloat();

    // Move the camera and animate the scene, scale movement with time step
    MoveCamera(timeStep);
    AnimateScene(timeStep);
}

void ParticleGraph::HandlePostRenderUpdate(StringHash eventType, VariantMap& eventData)
{
    // If draw debug mode is enabled, draw viewport debug geometry. This time use depth test, as otherwise the result
    // becomes hard to interpret due to large object count
    if (drawDebug_)
        GetSubsystem<Renderer>()->DrawDebugGeometry(true);
}