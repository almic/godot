#pragma once

class JoltConstraint {
public:
	virtual void build() = 0;
	virtual void destroy() = 0;
};
