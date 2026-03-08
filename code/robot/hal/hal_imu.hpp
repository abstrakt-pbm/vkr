#pragma once

namespace HAL {
	class IImuHAL {
	public:
		virtual float GetRawGyroX() const = 0;
		virtual float GetRawGyroY() const = 0;
		virtual float GetRawGyroZ() const = 0;

		virtual float GetAccelerationX() const = 0;
		virtual float GetAccelerationY() const = 0;
		virtual float GetAccelerationZ() const = 0;

		virtual bool IsAlive() const = 0;
	};
} // namespace HAL

