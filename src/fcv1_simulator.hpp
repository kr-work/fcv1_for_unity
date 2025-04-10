#include "box2d/box2d.h"
#include <cmath>
#include <nlohmann/json.hpp>

#ifdef _WIN32
  #define EXPORT_API extern "C" __declspec(dllexport)
#elif __linux__ || __APPLE__
  #define EXPORT_API extern "C" __attribute__((visibility("default")))
#endif

using json = nlohmann::json;


constexpr float kStoneRadius = 0.145f;
static constexpr ::uint8_t kStoneMax = 16;
constexpr float kPi = 3.14159265359f;
constexpr float cw = -kPi / 2.f;
constexpr float ccw = kPi / 2.f;
constexpr float y_lower_limit = 32.004f;
constexpr float y_upper_limit = 40.234f;
constexpr float x_upper_limit = 2.375f;
constexpr float x_lower_limit = -2.375f;
constexpr float stone_x_upper_limit = x_upper_limit - 2 * kStoneRadius;
constexpr float stone_x_lower_limit = x_lower_limit + 2 * kStoneRadius;
constexpr float tee_line = 38.405f;
constexpr float min_y = 30.0f;
constexpr float house_radius = 1.829f;
constexpr float EPSILON = std::numeric_limits<float>::epsilon();
constexpr size_t stones_per_simulation = 16;
constexpr size_t num_coordinates = 2;

struct Velocity
{
    b2Vec2 vel;
};

struct StonePosition
{
    float x;
    float y;
};

namespace digitalcurling3
{
    struct Vector2
    {
        float x; ///< x座標
        float y; ///< y座標

        /// \brief (0, 0)で初期化します
        constexpr Vector2() : x(0.f), y(0.f) {}

        /// \brief 指定された座標で初期化します
        ///
        /// \param[in] x x座標
        /// \param[in] y y座標
        constexpr Vector2(float x, float y) : x(x), y(y) {}

        /// \brief ベクトルの加算を行います
        ///
        /// \param[in] v ベクトル
        /// \returns このベクトル自身
        constexpr Vector2 &operator+=(Vector2 v)
        {
            x += v.x;
            y += v.y;
            return *this;
        }

        /// \brief ベクトルの減算を行います
        ///
        /// \param[in] v ベクトル
        /// \returns このベクトル自身
        constexpr Vector2 &operator-=(Vector2 v)
        {
            x -= v.x;
            y -= v.y;
            return *this;
        }

        /// \brief ベクトルにスカラー値を乗算します
        ///
        /// \param[in] f スカラー値
        /// \returns このベクトル自身
        constexpr Vector2 &operator*=(float f)
        {
            x *= f;
            y *= f;
            return *this;
        }

        /// \brief ベクトルをスカラー値で除算します
        ///
        /// \param[in] f スカラー値
        /// \returns このベクトル自身
        constexpr Vector2 &operator/=(float f)
        {
            x /= f;
            y /= f;
            return *this;
        }

        /// \brief ベクトルの長さを得ます
        ///
        /// \returns \code std::hypot(x, y) \endcode
        float Length() const
        {
            return std::hypot(x, y);
        }
    };

    /// \brief ベクトルを反転します
    ///
    /// \param[in] v ベクトル
    /// \returns \a -v
    constexpr Vector2 operator-(Vector2 v)
    {
        return {-v.x, -v.y};
    }

    /// \brief ベクトル同士の加算を行います
    ///
    /// \param[in] v1 ベクトル1
    /// \param[in] v2 ベクトル2
    /// \returns \a v1+v2
    constexpr Vector2 operator+(Vector2 v1, Vector2 v2)
    {
        return {v1.x + v2.x, v1.y + v2.y};
    }

    /// \brief ベクトル同士の減算を行います
    ///
    /// \param[in] v1 ベクトル1
    /// \param[in] v2 ベクトル2
    /// \returns \a v1-v2
    constexpr Vector2 operator-(Vector2 v1, Vector2 v2)
    {
        return {v1.x - v2.x, v1.y - v2.y};
    }

    /// \brief ベクトルとスカラー値の乗算を行います
    ///
    /// \param[in] f スカラー値
    /// \param[in] v ベクトル
    /// \returns \a f*v
    constexpr Vector2 operator*(float f, Vector2 v)
    {
        return {f * v.x, f * v.y};
    }

    /// \brief ベクトルとスカラー値の乗算を行います
    ///
    /// \param[in] v ベクトル
    /// \param[in] f スカラー値
    /// \returns \a v*f
    constexpr Vector2 operator*(Vector2 v, float f)
    {
        return f * v;
    }

    /// \brief ベクトルとスカラー値の除算を行います
    ///
    /// \param[in] v ベクトル
    /// \param[in] f スカラー値
    /// \returns \a v*(1/f)
    constexpr Vector2 operator/(Vector2 v, float f)
    {
        return {v.x / f, v.y / f};
    }

    inline b2Vec2 ToB2Vec2(Vector2 v)
    {
        return {v.x, v.y};
    }

    inline Vector2 ToDC2Vector2(b2Vec2 v)
    {
        return {v.x, v.y};
    }
}

namespace digitalcurling3
{
    /// \brief 位置，角度を格納します．
    struct Transform
    {
        Vector2 position; ///< 位置
        float angle;      ///< 角度

        /// \brief 位置(0, 0)，角度0で初期化します
        constexpr Transform() : position(), angle(0.f) {}

        /// \brief 指定された値で初期化します
        ///
        /// \param[in] position 位置
        /// \param[in] angle 角度
        constexpr Transform(Vector2 position, float angle) : position(position), angle(angle) {}
    };
}

namespace digitalcurling3
{
    /// \brief 位置を格納します．
    struct StoneData
    {
        Vector2 position;
        StoneData() {}
        StoneData(const Vector2 &pos) : position(pos) {}
    };
}

namespace digitalcurling3
{
    /// \brief 位置を格納します．
    struct StoneDataVector
    {
        std::vector<digitalcurling3::StoneData> stones;
    };
}

namespace digitalcurling3
{
    /// \brief five lockが適用されたかどうかのフラグとストーンIDを格納します．
    struct FiveLockWithID
    {
        unsigned int flag;
        int16_t id;
    };
}


namespace digitalcurling3
{
    /// \brief ストーンどうしの衝突の情報
    struct Collision
    {
        /// \brief 衝突に関するストーンの情報
        struct Stone
        {
            std::uint8_t id;     ///< ストーンのID
            Transform transform; ///< ストーンの位置と角度

            /// \brief デフォルトコンストラクタ
            Stone() : id(0), transform() {}

            /// \brief 与えられたデータで初期化します
            ///
            /// \param[in] id ストーンのID
            /// \param[in] transform ストーンの位置と角度
            Stone(std::uint8_t id, Transform const &transform) : id(id), transform(transform) {}
        };
        Stone a;               ///< 衝突したストーン
        Stone b;               ///< 衝突したストーン
        float normal_impulse;  ///< 法線方向の撃力
        float tangent_impulse; ///< 接線方向の撃力
        /// \brief 全パラメータを 0 で初期化します
        Collision()
            : a(), b(), normal_impulse(0.f), tangent_impulse(0.f) {}
        /// \brief 与えられたパラメータで初期化します
        ///
        /// \param[in] a_id ストーンAのID
        /// \param[in] b_id ストーンBのID
        /// \param[in] a_transform ストーンAの位置
        /// \param[in] b_transform ストーンBの位置
        /// \param[in] normal_impulse 法線方向の撃力
        /// \param[in] tangent_impulse 接線方向の撃力
        Collision(std::uint8_t a_id, std::uint8_t b_id, Transform const &a_transform, Transform const &b_transform, float normal_impulse, float tangent_impulse)
            : a(a_id, a_transform), b(b_id, b_transform), normal_impulse(normal_impulse), tangent_impulse(tangent_impulse) {}

        /// \brief ストーンどうしが接した座標を得る。
        ///
        /// \returns ストーンどうしが接した座標
        Vector2 GetContactPoint() const
        {
            return (a.transform.position + b.transform.position) * 0.5f;
        }
    };
}

class StoneData
{
public:
    b2Body *body;
    std::vector<digitalcurling3::Collision> collisions;
};

class SimulatorFCV1
{
public:
    explicit SimulatorFCV1();
    class ContactListener : public b2ContactListener
    {
    public:
        ContactListener(SimulatorFCV1 *instance) : instance_(instance) {}
        virtual void PostSolve(b2Contact *contact, const b2ContactImpulse *impulse) override;
        void add_unique_id(std::vector<int> &list, int id);

    private:
        SimulatorFCV1 *const instance_;
    };
    bool is_freeguardzone(b2Body *body);
    void freeguardzone_checker();
    void is_in_playarea();
    bool on_center_line(b2Body *body);
    void no_tick_checker();
    void no_tick_rule();
    bool step(int stone_id = -1, float coefficient = 1.0f);
    void set_stones();
    void reset_stones();
    void set_velocity(float velocity_x, float velocity_y, float angular_velocity, unsigned int total_shot,unsigned int shot_per_team, unsigned int team_id);
    void get_stones();
    void set_stone_position_buffer(digitalcurling3::StoneData *stone_position_buffer);
    void set_status(int status);

private:
    ContactListener contact_listener_;
    std::vector<digitalcurling3::StoneData> stones;
    int shot_per_team;
    float angular_velocity;
    std::vector<int> is_awake;
    std::vector<int> is_no_tick;
    std::vector<int> in_free_guard_zone;
    digitalcurling3::FiveLockWithID five_lock_with_id;
    float new_stone_speed;
    std::vector<digitalcurling3::StoneData> current_positions;
    unsigned int total_shot;
    bool free_guard_zone;
    b2World world;
    b2BodyDef stone_body_def;
    std::array<b2Body *, static_cast<std::size_t>(kStoneMax)> stone_bodies;
    digitalcurling3::StoneData* stone_position_buffer = nullptr;
    int status = 0; // 0: five lock, 1: no tick
};

/// @brief Create a "SimulatorFCV1" plugin instance
/// @param stone_data This is a pointer to the stone data buffer. Mainly used to set and get the stone position.
/// @return "SimulatorFCV1" plugin instance
EXPORT_API SimulatorFCV1* create_plugin(digitalcurling3::StoneData* stone_data);

/// @brief Destroy the "SimulatorFCV1" plugin instance
/// @param plugin Simulator plugin instance
/// @return 
EXPORT_API void destroy_plugin(SimulatorFCV1* plugin);

/// @brief Reset the stone position.
/// @param plugin Simulator plugin instance.
/// @return 
EXPORT_API void reset_stones(SimulatorFCV1* plugin);

/// @brief Set the stone position.
/// @param plugin Simulator plugin instance.
/// @return 
EXPORT_API void set_stones(SimulatorFCV1* plugin);

/// @brief Set the stone velocity.
/// @param plugin Simulator plugin instance.
/// @param velocity_x The x component of the stone velocity.
/// @param velocity_y The y component of the stone velocity.
/// @param angular_velocity The angular velocity of the stone.
/// @param total_shot The total shot of the "end"
/// @param shot_per_team The number of shots per team.
/// @param team_id 0: Team0, 1: Team1
/// @return 
EXPORT_API void set_velocity(SimulatorFCV1* plugin, float velocity_x, float velocity_y, float angular_velocity, int total_shot, unsigned int shot_per_team, unsigned int team_id);

/// @brief Set the status of the plugin.
/// @param plugin simulator plugin instance.
/// @param status 0: five lock, 1: no tick
/// @return 
EXPORT_API void set_status(SimulatorFCV1* plugin, int status);

/// @brief Get the stone position. The stone position is stored in the "stone_position_buffer" of the plugin.
/// @param plugin
/// @return
EXPORT_API void check_rule(SimulatorFCV1* plugin);

/// @brief Simulate the stones.
/// @param plugin 
/// @param index THis is stone id. "Team0" is 0 to 7 and "Team1" is 8 to 15.
/// @param coefficient This is the coefficient of the "dynamic friction coefficient", which is difficult to apply in the simulation directly, so it is multiplied by this coefficient.
/// @return
EXPORT_API bool step(SimulatorFCV1* plugin, int index, float coefficient);