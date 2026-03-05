#pragma once

namespace HAL {
	class IEncoderHAL {
	public:
    	virtual float GetRawLinearVelocity() const = 0;
		virtual bool IsAlive() const = 0;
	};
} // namespace HAL

