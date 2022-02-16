/*
* This file is automatically generated by px_generate_mixers.py - do not edit.
*/

#ifndef _AVIATA_MIXER_MULTI_TABLES
#define _AVIATA_MIXER_MULTI_TABLES

#define AVIATA_MAX_NUM_DRONES 4
#define AVIATA_MAX_NUM_ROTORS 6

#ifdef NEED_MIXER_TYPES
typedef uint16_t MultirotorGeometryUnderlyingType;

namespace MultirotorMixer {
	struct Rotor {
		float	roll_scale;		/**< scales roll for this rotor */
		float	pitch_scale;	/**< scales pitch for this rotor */
		float	yaw_scale;		/**< scales yaw for this rotor */
		float	thrust_scale;	/**< scales thrust for this rotor */
	};
}
#endif

enum AviataAirframe {
	AVIATA_4,
	AVIATA_4_ALT,
	AVIATA_2,
	AVIATA_NUM_AIRFRAMES
};

struct AviataFrameInfo {
	MultirotorGeometryUnderlyingType start_index;
	uint8_t num_drones;
	uint8_t num_rotors;
	uint8_t max_missing_drones;
	float drone_angle[AVIATA_MAX_NUM_DRONES];
	float drone_angle_cos[AVIATA_MAX_NUM_DRONES];
	float drone_angle_sin[AVIATA_MAX_NUM_DRONES];
	float relative_drone_angle[AVIATA_MAX_NUM_DRONES][AVIATA_MAX_NUM_DRONES];
	float relative_drone_angle_cos[AVIATA_MAX_NUM_DRONES][AVIATA_MAX_NUM_DRONES];
	float relative_drone_angle_sin[AVIATA_MAX_NUM_DRONES][AVIATA_MAX_NUM_DRONES];
};

enum class AviataMultirotorGeometry : MultirotorGeometryUnderlyingType {
	AVIATA_4_MISSING_,             // aviata_4 with these drones missing:  (text key aviata_4_missing_)
	AVIATA_4_ALT_MISSING_,         // aviata_4_alt with these drones missing:  (text key aviata_4_alt_missing_)
	AVIATA_2_MISSING_,             // aviata_2 with these drones missing:  (text key aviata_2_missing_)

	MAX_GEOMETRY
}; // enum class AviataMultirotorGeometry

static constexpr AviataFrameInfo _config_aviata_frame_info[AVIATA_NUM_AIRFRAMES] {
	/*[AVIATA_4] =*/ {
		/*.start_index =*/ (MultirotorGeometryUnderlyingType) AviataMultirotorGeometry::AVIATA_4_MISSING_,
		/*.num_drones =*/ 4,
		/*.num_rotors =*/ 6,
		/*.max_missing_drones =*/ 0,
		/*.drone_angle =*/ {
			 0.785398, // 45.0 degrees
			 2.356194, // 135.0 degrees
			-2.356194, // -135.0 degrees
			-0.785398, // -45.0 degrees
		},
		/*.drone_angle_cos =*/ {
			 0.707107,
			-0.707107,
			-0.707107,
			 0.707107,
		},
		/*.drone_angle_sin =*/ {
			 0.707107,
			 0.707107,
			-0.707107,
			-0.707107,
		},
		/*.relative_drone_angle =*/ {
			{  0.000000, -1.570796,  3.141593,  1.570796, },
			{  1.570796,  0.000000, -1.570796,  3.141593, },
			{ -3.141593,  1.570796,  0.000000, -1.570796, },
			{ -1.570796, -3.141593,  1.570796,  0.000000, },
		},
		/*.relative_drone_angle_cos =*/ {
			{  1.000000,  0.000000, -1.000000,  0.000000, },
			{  0.000000,  1.000000, -0.000000, -1.000000, },
			{ -1.000000, -0.000000,  1.000000,  0.000000, },
			{  0.000000, -1.000000,  0.000000,  1.000000, },
		},
		/*.relative_drone_angle_sin =*/ {
			{  0.000000, -1.000000,  0.000000,  1.000000, },
			{  1.000000,  0.000000, -1.000000,  0.000000, },
			{ -0.000000,  1.000000,  0.000000, -1.000000, },
			{ -1.000000, -0.000000,  1.000000,  0.000000, },
		},
	},

	/*[AVIATA_4_ALT] =*/ {
		/*.start_index =*/ (MultirotorGeometryUnderlyingType) AviataMultirotorGeometry::AVIATA_4_ALT_MISSING_,
		/*.num_drones =*/ 4,
		/*.num_rotors =*/ 6,
		/*.max_missing_drones =*/ 0,
		/*.drone_angle =*/ {
			 0.785398, // 45.0 degrees
			 2.356194, // 135.0 degrees
			-2.356194, // -135.0 degrees
			-0.785398, // -45.0 degrees
		},
		/*.drone_angle_cos =*/ {
			 0.707107,
			-0.707107,
			-0.707107,
			 0.707107,
		},
		/*.drone_angle_sin =*/ {
			 0.707107,
			 0.707107,
			-0.707107,
			-0.707107,
		},
		/*.relative_drone_angle =*/ {
			{  0.000000, -1.570796,  3.141593,  1.570796, },
			{  1.570796,  0.000000, -1.570796,  3.141593, },
			{ -3.141593,  1.570796,  0.000000, -1.570796, },
			{ -1.570796, -3.141593,  1.570796,  0.000000, },
		},
		/*.relative_drone_angle_cos =*/ {
			{  1.000000,  0.000000, -1.000000,  0.000000, },
			{  0.000000,  1.000000, -0.000000, -1.000000, },
			{ -1.000000, -0.000000,  1.000000,  0.000000, },
			{  0.000000, -1.000000,  0.000000,  1.000000, },
		},
		/*.relative_drone_angle_sin =*/ {
			{  0.000000, -1.000000,  0.000000,  1.000000, },
			{  1.000000,  0.000000, -1.000000,  0.000000, },
			{ -0.000000,  1.000000,  0.000000, -1.000000, },
			{ -1.000000, -0.000000,  1.000000,  0.000000, },
		},
	},

	/*[AVIATA_2] =*/ {
		/*.start_index =*/ (MultirotorGeometryUnderlyingType) AviataMultirotorGeometry::AVIATA_2_MISSING_,
		/*.num_drones =*/ 2,
		/*.num_rotors =*/ 6,
		/*.max_missing_drones =*/ 0,
		/*.drone_angle =*/ {
			 0.000000, // 0.0 degrees
			-0.785398, // -45.0 degrees
		},
		/*.drone_angle_cos =*/ {
			 1.000000,
			 0.707107,
		},
		/*.drone_angle_sin =*/ {
			 0.000000,
			-0.707107,
		},
		/*.relative_drone_angle =*/ {
			{  0.000000,  0.785398, },
			{ -0.785398,  0.000000, },
		},
		/*.relative_drone_angle_cos =*/ {
			{  1.000000,  0.707107, },
			{  0.707107,  1.000000, },
		},
		/*.relative_drone_angle_sin =*/ {
			{  0.000000,  0.707107, },
			{ -0.707107,  0.000000, },
		},
	},

};

namespace {
static constexpr MultirotorMixer::Rotor _config_aviata_aviata_4_missing_[] {
	{ -0.030201,  0.030201, -0.032942,  0.774201 },
	{ -0.030201,  0.030201,  0.032942,  0.774201 },
	{ -0.030201,  0.030201, -0.413627,  0.774201 },
	{ -0.030201,  0.030201, -0.413627,  0.774201 },
	{ -0.030201,  0.030201,  0.413627,  0.774201 },
	{ -0.030201,  0.030201,  0.413627,  0.774201 },
	{ -0.030201, -0.030201, -0.032942,  0.774201 },
	{ -0.030201, -0.030201,  0.032942,  0.774201 },
	{ -0.030201, -0.030201, -0.413627,  0.774201 },
	{ -0.030201, -0.030201, -0.413627,  0.774201 },
	{ -0.030201, -0.030201,  0.413627,  0.774201 },
	{ -0.030201, -0.030201,  0.413627,  0.774201 },
	{  0.030201, -0.030201, -0.032942,  0.774201 },
	{  0.030201, -0.030201,  0.032942,  0.774201 },
	{  0.030201, -0.030201, -0.413627,  0.774201 },
	{  0.030201, -0.030201, -0.413627,  0.774201 },
	{  0.030201, -0.030201,  0.413627,  0.774201 },
	{  0.030201, -0.030201,  0.413627,  0.774201 },
	{  0.030201,  0.030201, -0.032942,  0.774201 },
	{  0.030201,  0.030201,  0.032942,  0.774201 },
	{  0.030201,  0.030201, -0.413627,  0.774201 },
	{  0.030201,  0.030201, -0.413627,  0.774201 },
	{  0.030201,  0.030201,  0.413627,  0.774201 },
	{  0.030201,  0.030201,  0.413627,  0.774201 },
};

static constexpr MultirotorMixer::Rotor _config_aviata_aviata_4_alt_missing_[] {
	{ -0.030201,  0.030201,  0.028526,  0.774201 },
	{ -0.030201,  0.030201, -0.028526,  0.774201 },
	{ -0.030201,  0.030201, -0.424202,  0.774201 },
	{ -0.030201,  0.030201, -0.424202,  0.774201 },
	{ -0.030201,  0.030201,  0.424202,  0.774201 },
	{ -0.030201,  0.030201,  0.424202,  0.774201 },
	{ -0.030201, -0.030201,  0.028526,  0.774201 },
	{ -0.030201, -0.030201, -0.028526,  0.774201 },
	{ -0.030201, -0.030201, -0.424202,  0.774201 },
	{ -0.030201, -0.030201, -0.424202,  0.774201 },
	{ -0.030201, -0.030201,  0.424202,  0.774201 },
	{ -0.030201, -0.030201,  0.424202,  0.774201 },
	{  0.030201, -0.030201,  0.028526,  0.774201 },
	{  0.030201, -0.030201, -0.028526,  0.774201 },
	{  0.030201, -0.030201, -0.424202,  0.774201 },
	{  0.030201, -0.030201, -0.424202,  0.774201 },
	{  0.030201, -0.030201,  0.424202,  0.774201 },
	{  0.030201, -0.030201,  0.424202,  0.774201 },
	{  0.030201,  0.030201,  0.028526,  0.774201 },
	{  0.030201,  0.030201, -0.028526,  0.774201 },
	{  0.030201,  0.030201, -0.424202,  0.774201 },
	{  0.030201,  0.030201, -0.424202,  0.774201 },
	{  0.030201,  0.030201,  0.424202,  0.774201 },
	{  0.030201,  0.030201,  0.424202,  0.774201 },
};

static constexpr MultirotorMixer::Rotor _config_aviata_aviata_2_missing_[] {
	{ -0.030069, -0.004809, -0.149920,  1.081320 },
	{ -0.030069,  0.004809,  0.149920,  1.081320 },
	{ -0.030069,  0.021088, -0.149920,  1.081320 },
	{ -0.030069, -0.021088,  0.149920,  1.081320 },
	{ -0.030069,  0.021088,  0.149920,  1.081320 },
	{ -0.030069, -0.021088, -0.149920,  1.081320 },
	{  0.030069,  0.021088, -0.149920,  1.081320 },
	{  0.030069, -0.021088,  0.149920,  1.081320 },
	{  0.030069,  0.004809, -0.149920,  1.081320 },
	{  0.030069, -0.004809,  0.149920,  1.081320 },
	{  0.030069,  0.021088,  0.149920,  1.081320 },
	{  0.030069, -0.021088, -0.149920,  1.081320 },
};

static constexpr const MultirotorMixer::Rotor *_config_aviata_index[] {
	&_config_aviata_aviata_4_missing_[0],
	&_config_aviata_aviata_4_alt_missing_[0],
	&_config_aviata_aviata_2_missing_[0],
};

static constexpr unsigned _config_aviata_rotor_count[] {
	24, /* aviata_4_missing_ */
	24, /* aviata_4_alt_missing_ */
	12, /* aviata_2_missing_ */
};

__attribute__((unused)) // Not really unused, but fixes compilation error
const char* _config_aviata_key[] {
	"aviata_4_missing_",	/* aviata_4_missing_ */
	"aviata_4_alt_missing_",	/* aviata_4_alt_missing_ */
	"aviata_2_missing_",	/* aviata_2_missing_ */
};

} // anonymous namespace

#endif /* _AVIATA_MIXER_MULTI_TABLES */

