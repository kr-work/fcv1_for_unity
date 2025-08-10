#include <stdio.h>
#include <iostream>
#include <nlohmann/json.hpp>
#include <box2d/box2d.h>
#include <cmath>
#include <limits>
#include "fcv1_simulator.hpp"
#include <set>
#include <string>
#include <fstream>
#include <algorithm>


// constexpr float stone_radius = 0.145f;

json read_configfile(const std::string &filepath)
{
    std::ifstream ifs(filepath);
    json j;
    ifs >> j;
    return j;
}

// 返り値1つめ: 正規化されたベクトル
// 返り値2つめ: もとのベクトルの長さ
/// \brief To normalize the vector
/// \param[in] v The vector to be normalized
/// \returns A pair of the normalized vector and the length of the original vector
inline std::pair<b2Vec2, float> normalize(b2Vec2 const &v)
{
    b2Vec2 normalized = v;
    float length = normalized.Normalize();
    return {normalized, length};
}

/// \brief To calculate the longitudinal acceleration
/// \param[in] speed The speed of the stone
/// \returns The longitudinal acceleration
inline float longitudinal_acceleration(float speed)
{
    constexpr float kGravity = 9.80665f;
    return -(0.00200985f / (speed + 0.06385782f) + 0.00626286f) * kGravity;
}

/// \brief To calculate the yaw rate
/// \param[in] speed The speed of the stone
/// \param[in] angularVelocity The angular velocity of the stone
/// \returns The yaw rate
inline float yaw_rate(float speed, float angularVelocity)
{
    if (std::abs(angularVelocity) <= EPSILON)
    {
        return 0.f;
    }
    return (angularVelocity > 0.f ? 1.0f : -1.0f) * 0.00820f * std::pow(speed, -0.8f);
}

/// \brief To calculate the angular acceleration
/// \param[in] linearSpeed The speed of the stone
/// \returns The angular acceleration
inline float angular_acceleration(float linearSpeed)
{
    float clampedSpeed = std::max(linearSpeed, 0.001f);
    return -0.025f / clampedSpeed;
}

void SimulatorFCV1::ContactListener::PostSolve(b2Contact *contact, const b2ContactImpulse *impulse)
{
    b2Body *a_body = contact->GetFixtureA()->GetBody();
    b2Body *b_body = contact->GetFixtureB()->GetBody();

    digitalcurling3::Collision collision;
    collision.a.id = static_cast<int>(a_body->GetUserData().pointer);
    collision.b.id = static_cast<int>(b_body->GetUserData().pointer);

    instance_->step_result.is_collision_occured = 1;

    add_unique_id(instance_->is_awake, collision.a.id);
    add_unique_id(instance_->is_awake, collision.b.id);

    b2WorldManifold world_manifold;
    contact->GetWorldManifold(&world_manifold);

    collision.normal_impulse = impulse->normalImpulses[0];
    collision.tangent_impulse = impulse->tangentImpulses[0];
}

void SimulatorFCV1::ContactListener::add_unique_id(std::vector<int> &list, int id)
{
    if (std::find(list.begin(), list.end(), id) == list.end())
    {
        list.push_back(id);
    }
}

SimulatorFCV1::SimulatorFCV1() : world(b2Vec2(0, 0)), contact_listener_(this)
{
    stones.resize(kStoneMax);
    for (size_t i = 0; i < kStoneMax; ++i)
    {
        digitalcurling3::Vector2 position = {0.0f, 0.0f};
        stones[i].position = position;
    }

    stone_body_def.type = b2_dynamicBody;
    stone_body_def.awake = false;
    stone_body_def.bullet = true;
    stone_body_def.enabled = false;

    b2CircleShape stone_shape;
    stone_shape.m_radius = kStoneRadius;

    b2FixtureDef stone_fixture_def;
    stone_fixture_def.shape = &stone_shape;
    stone_fixture_def.friction = 0.2f;                                        // 適当というかデフォルト値
    stone_fixture_def.restitution = 1.0;                                      // 完全弾性衝突(完全弾性衝突の根拠は無いし多分違う)
    stone_fixture_def.restitutionThreshold = 0.f;                             // 反発閾値。この値より大きい速度(m/s)で衝突すると反発が適用される。
    stone_fixture_def.density = 0.5f / (b2_pi * kStoneRadius * kStoneRadius); // kg/m^2

    for (size_t i = 0; i < kStoneMax; ++i)
    {
        stone_body_def.userData.pointer = static_cast<uintptr_t>(i);
        stone_bodies[i] = world.CreateBody(&stone_body_def);
        stone_bodies[i]->CreateFixture(&stone_fixture_def);
    }
    world.SetContactListener(&contact_listener_);
}

bool SimulatorFCV1::is_freeguardzone(b2Body *body)
{
    float dx = body->GetPosition().x;
    float dy = body->GetPosition().y - tee_line;
    float distance_squared = dx * dx + dy * dy;
    if (dy < 0 && distance_squared > house_radius * house_radius && body->GetPosition().y >= min_y)
    {
        return true;
    }
    return false;
}

void SimulatorFCV1::freeguardzone_checker()
{
    for (size_t i = 0; i < kStoneMax; ++i)
    {
        b2Body *body = stone_bodies[i];
        if (is_freeguardzone(body))
        {
            in_free_guard_zone.push_back(i);
        }
    }
}

// ファイブロックルール対応用関数
void SimulatorFCV1::is_in_playarea()
{
    for (int i : in_free_guard_zone)
    {
        b2Body *body = stone_bodies[i];
        float position_x = body->GetPosition().x;
        float position_y = body->GetPosition().y;
        if (position_y > y_upper_limit || position_x > stone_x_upper_limit || position_x < stone_x_lower_limit || (position_x == 0.0f && position_y == 0.0f))
        {
            for (int i = 0; i < kStoneMax; ++i)
            {
                digitalcurling3::StoneData stone = stones[i];
                stone_bodies[i]->SetTransform(b2Vec2(stone.position.x, stone.position.y), 0.f);
                if (stone.position.x == 0.f && stone.position.y == 0.f)
                {
                    stone_bodies[i]->SetEnabled(false);
                    stone_bodies[i]->SetAwake(false);
                }
                else
                {
                    stone_bodies[i]->SetEnabled(true);
                    stone_bodies[i]->SetAwake(true);
                }
            }
        }
    }
}

// ノーティックルール対応用関数
bool SimulatorFCV1::on_center_line(b2Body *body)
{
    if (std::abs(body->GetPosition().x) <= kStoneRadius)
    {
        return true;
    }
    return false;
}

void SimulatorFCV1::no_tick_checker()
{
    for (size_t i = 0; i < kStoneMax; ++i)
    {
        b2Body *body = stone_bodies[i];
        float position_y = body -> GetPosition().y;
        if (position_y > y_lower_limit && position_y < (tee_line - house_radius) && on_center_line(body))
        {
            is_no_tick.push_back(i);
        }
    }
}

// ノーティックルール対応用関数
void SimulatorFCV1::no_tick_rule()
{
    for (int i : is_no_tick)
    {
        b2Body *body = stone_bodies[i];
        float position_x = body->GetPosition().x;
        float position_y = body->GetPosition().y;
        if (std::abs(position_x) > kStoneRadius || (position_x == 0.0f && position_y == 0.0f))
        {
            for (size_t j = 0; j < kStoneMax; ++j)
            {
                digitalcurling3::StoneData stone = stones[j];
                stone_bodies[j]->SetTransform(b2Vec2(stone.position.x, stone.position.y), 0.f);
            }
            break;
        }
    }
}

StepResult SimulatorFCV1::step(int stone_id, float coefficient)
{
    // simulate
    for (int &i : is_awake)
    {
        b2Vec2 const stone_velocity = stone_bodies[i]->GetLinearVelocity();
        auto const [normalized_stone_velocity, stone_speed] = normalize(stone_velocity);
        float const angular_velocity = stone_bodies[i]->GetAngularVelocity();

        // 速度を計算
        // ストーンが停止してる場合は無視
        if (stone_speed > EPSILON)
        {
            digitalcurling3::Vector2 stone_position = {stone_bodies[i]->GetPosition().x, stone_bodies[i]->GetPosition().y};
            if (stone_position.x > stone_x_upper_limit || stone_x_lower_limit > stone_position.x)
            {
                stone_bodies[i]->SetTransform(b2Vec2(0.f, 0.f), 0.f);
                stone_bodies[i]->SetAngularVelocity(0.0f);
                stone_bodies[i]->SetAwake(false);
                stone_bodies[i]->SetEnabled(false);
                is_awake.erase(std::remove(is_awake.begin(), is_awake.end(), i), is_awake.end());
                continue;
            }
            // ストーンの速度を計算
            if (i == stone_id)
            {
                new_stone_speed = stone_speed + longitudinal_acceleration(stone_speed) * 0.002f * coefficient;
            } else
            {
                new_stone_speed = stone_speed + longitudinal_acceleration(stone_speed) * 0.002f;
            }
            if (new_stone_speed <= 0.f)
            {
                stone_bodies[i]->SetLinearVelocity(b2Vec2_zero);
                is_awake.erase(std::remove(is_awake.begin(), is_awake.end(), i), is_awake.end());
            }
            else
            {
                float yaw = 0.f;
                if (this->index == i && this->shot_status == 1)
                {
                    yaw = 0.f;
                }
                else
                {
                    yaw = yaw_rate(stone_speed, angular_velocity) * 0.002;
                }
                float const longitudinal_velocity = new_stone_speed * std::cos(yaw);
                float const transverse_velocity = new_stone_speed * std::sin(yaw);
                b2Vec2 const &e_longitudinal = normalized_stone_velocity;
                b2Vec2 const e_transverse = e_longitudinal.Skew();
                b2Vec2 const new_stone_velocity = longitudinal_velocity * e_longitudinal + transverse_velocity * e_transverse;
                stone_bodies[i]->SetLinearVelocity(new_stone_velocity);
            }
        }else{
            is_awake.erase(std::remove(is_awake.begin(), is_awake.end(), i), is_awake.end());
        }

        // 角速度を計算
        if (std::abs(angular_velocity) > EPSILON)
        {
            float const angular_accel = angular_acceleration(stone_speed) * 0.002;
            float new_angular_velocity = 0.f;
            if (std::abs(angular_velocity) <= std::abs(angular_accel))
            {
                new_angular_velocity = 0.f;
            }
            else
            {
                new_angular_velocity = angular_velocity + angular_accel * angular_velocity / std::abs(angular_velocity);
            }
            stone_bodies[i]->SetAngularVelocity(new_angular_velocity);
        }
    }

    // storage.collisions.clear();

    world.Step(
        0.002,
        8,  // velocityIterations (公式マニュアルでの推奨値は 8)
        3); // positionIterations (公式マニュアルでの推奨値は 3)
    
    if (is_awake.empty())
    {
        step_result.calculating = 0;
        step_result.thrown_stone_angular_velocity = 0.0f;
        for (size_t i = 0; i < kStoneMax; ++i)
        {
            stone_bodies[i]->SetAngularVelocity(0.0f);
            b2Vec2 position = stone_bodies[i]->GetPosition();
            digitalcurling3::Vector2 stone_position = {position.x, position.y};
            this->stone_position_buffer[i] = stone_position;
            this->stone_angular_velocity_buffer[i].angular_velocity = stone_bodies[i]->GetAngularVelocity();
        }
        return step_result;
    }


    for (int j: is_awake)
    {
        b2Vec2 position = stone_bodies[j]->GetPosition();
        digitalcurling3::Vector2 stone_position = {position.x, position.y};
        this->stone_position_buffer[j] = stone_position;
        this->stone_angular_velocity_buffer[j].angular_velocity = stone_bodies[j]->GetAngularVelocity();
    }
    step_result.calculating = 1;
    step_result.thrown_stone_angular_velocity = stone_bodies[stone_id]->GetAngularVelocity();
    return step_result;
}

void SimulatorFCV1::reset_stones()
{
    for (size_t i = 0; i < kStoneMax; i++)
    {
        b2Body *body = stone_bodies[i];
        body->SetTransform(b2Vec2(0.f, 0.f), 0.f);
        body->SetAwake(false);
        body->SetEnabled(false);
        this->stone_position_buffer[i] = digitalcurling3::Vector2(0.0f, 0.0f);
    }
}

void SimulatorFCV1::set_stones()
{
    // update bodies
    for (size_t i = 0; i < kStoneMax; ++i)
    {
        digitalcurling3::Vector2 position = this->stone_position_buffer[i].position;
        if (position.x == 0.f && position.y == 0.f)
        {
            stone_bodies[i]->SetEnabled(false);
        }
        else
        {
            stone_bodies[i]->SetEnabled(true);
            stone_bodies[i]->SetAwake(true);
            stone_bodies[i]->SetTransform(b2Vec2(position.x, position.y), 0.f);
        }
    }
}

/// @brief Function to call simulate stones
/// @param velocity_x The x component of the velocity of the stone to be thrown 
/// @param velocity_y The y component of the velocity of the stone to be thrown
/// @param angular_velocity The angular velocity of the stone to be thrown
/// @param shot_per_team The number of shots per team, which is 0 to 7
/// @param team_id The team that throws the stone. The number that "Team0" is 0 and "Team1" is 1. Team0 is the first attacker at the first end.
void SimulatorFCV1::set_velocity(float velocity_x, float velocity_y, float angular_velocity, unsigned int total_shot, unsigned int shot_per_team, unsigned int team_id, unsigned int shot_status)
{
    is_awake.clear();
    in_free_guard_zone.clear();
    is_no_tick.clear();

    this->total_shot = total_shot;
    this->shot_per_team = shot_per_team;
    this->index = this->shot_per_team + team_id * 8;
    this->shot_status = shot_status;
    stone_bodies[this->index]->SetLinearVelocity(b2Vec2(velocity_x, velocity_y));
    stone_bodies[this->index]->SetAngularVelocity(angular_velocity);
    stone_bodies[this->index]->SetEnabled(true);
    stone_bodies[this->index]->SetAwake(true);
    stone_bodies[this->index]->SetTransform(b2Vec2(0.0f, 0.0f), 0.f);
    is_awake.push_back(this->index);
    for (size_t i = 0; i < kStoneMax; ++i)
    {
        b2Vec2 position = stone_bodies[i]->GetPosition();
        stones[i].position = digitalcurling3::Vector2(position.x, position.y);
    }

    if (this->total_shot < 5)
    {
        if (this->status == 0)    // status=0: apply five rock rule
        {
            freeguardzone_checker();
        }
        else if (this->status == 1) // status=1: apply no tick rule
        {
            no_tick_checker();
        }
    }
}

void SimulatorFCV1::set_status(int status)
{
    this->status = status;
}

void SimulatorFCV1::get_stones()
{
    if (this->total_shot < 5)
    {
        if (this->status == 0)
        {
            is_in_playarea();
        }
        else if (this->status == 1)
        {
            no_tick_rule();
        }
    }

    for (size_t i = 0; i < kStoneMax; i++)
    {
        b2Body *body = stone_bodies[i];
        b2Vec2 position = body->GetPosition();
        if (position.x > stone_x_upper_limit || position.x < stone_x_lower_limit || position.y > y_upper_limit || position.y < y_lower_limit)  // This stone is out of the play area
        {
            body->SetTransform(b2Vec2(0.f, 0.f), 0.f);
            body->SetAwake(false);
            body->SetEnabled(false);
        }
        b2Vec2 after_position = body->GetPosition();
        this->stone_position_buffer[i] = digitalcurling3::Vector2(after_position.x, after_position.y);
    }
}

void SimulatorFCV1::set_stone_buffer(digitalcurling3::StoneData *stone_position_buffer, digitalcurling3::StoneAngularVelocity *stone_angular_velocity_buffer)
{
    this->stone_position_buffer = stone_position_buffer;
    this->stone_angular_velocity_buffer = stone_angular_velocity_buffer;
}

/// @brief Create a "SimulatorFCV1" plugin instance
/// @param stone_data This is a pointer to the stone data buffer. Mainly used to set and get the stone position.
/// @return "SimulatorFCV1" plugin instance
SimulatorFCV1* create_plugin(digitalcurling3::StoneData* stone_data, digitalcurling3::StoneAngularVelocity* stone_angular_velocity_data)
{
    SimulatorFCV1* plugin = new SimulatorFCV1();
    plugin->set_stone_buffer(stone_data, stone_angular_velocity_data);
    return plugin;
}

/// @brief Destroy the "SimulatorFCV1" plugin instance
/// @param plugin Simulator plugin instance
/// @return 
void destroy_plugin(SimulatorFCV1* plugin)
{
    delete plugin;
}

/// @brief Reset the stone position.
/// @param plugin Simulator plugin instance.
/// @return 
void reset_stones(SimulatorFCV1* plugin)
{
    plugin->reset_stones();
}

/// @brief Set the stone position.
/// @param plugin Simulator plugin instance.
/// @return 
void set_stones(SimulatorFCV1* plugin)
{
    plugin->set_stones();
}

/// @brief Set the stone velocity.
/// @param plugin Simulator plugin instance.
/// @param velocity_x The x component of the stone velocity.
/// @param velocity_y The y component of the stone velocity.
/// @param angular_velocity The angular velocity of the stone.
/// @param total_shot The total shot of the "end"
/// @param shot_per_team The number of shots per team.
/// @param team_id 0: Team0, 1: Team1
/// @param shot_status 0: draw shot, 1: takeout shot
/// @return 
void set_velocity(SimulatorFCV1* plugin, float velocity_x, float velocity_y, float angular_velocity, int total_shot, unsigned int shot_per_team, unsigned int team_id, unsigned int shot_status)
{
    plugin->set_velocity(velocity_x, velocity_y, angular_velocity, total_shot,shot_per_team, team_id, shot_status);
}

/// @brief Set the status of the plugin.
/// @param plugin simulator plugin instance.
/// @param status 0: five lock, 1: no tick
/// @return 
void set_status(SimulatorFCV1* plugin, int status)
{
    plugin->set_status(status);
}

/// @brief Get the stone position. The stone position is stored in the "stone_position_buffer" of the plugin.
/// @param plugin
/// @return
void check_rule(SimulatorFCV1* plugin)
{
    plugin->get_stones();
}

/// @brief Simulate the stones.
/// @param plugin 
/// @param index THis is stone id. "Team0" is 0 to 7 and "Team1" is 8 to 15.
/// @param coefficient This is the coefficient of the "dynamic friction coefficient", which is difficult to apply in the simulation directly, so it is multiplied by this coefficient.
/// @return
void step(SimulatorFCV1* plugin, int index, float coefficient, StepResult* step_result)
{
    *step_result = plugin->step(index, coefficient);
}
