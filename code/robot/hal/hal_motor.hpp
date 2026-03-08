#pragma once

namespace HAL {
	class IMotorHAL {
	public:
		virtual bool SetRawVoltage(float voltage) = 0;

		virtual float GetCurrentRawVoltage() const = 0;
		virtual float GetMinVoltageToStart() const = 0;
		virtual float GetMaxVoltage() const = 0;

		virtual bool IsAlive() const = 0;
	};
} // namespace HAL

