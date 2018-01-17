#include "schd.h"

#define MAX_SCHD_ITEMS 32

struct SchdItem {
	int timeout_microsecond;
	SchdCallback callback;
	union SchdParameter param;
};

struct SchdItem _schd_items[MAX_SCHD_ITEMS] = { 0 };
int _schd_last_microsecond = 0;

int Schd_After(int microsecond, SchdCallback callback, union SchdParameter p) {
	int idx = 0;
	for (idx = 0; idx < MAX_SCHD_ITEMS; ++idx) {
		if (_schd_items[idx].timeout_microsecond == 0) {
			_schd_items[idx].timeout_microsecond = _schd_last_microsecond
					+ microsecond;
			_schd_items[idx].callback = callback;
			_schd_items[idx].param = p;
			return 1;
		}
	}
	return 0;
}

int Schd_After_Int(int microsecond, SchdCallback callback, unsigned char value) {
	union SchdParameter p;
	p.data = value;
	return Schd_After(microsecond, callback, p);
}

void Schd_Run(int current_microsecond) {
	int idx = 0;
	_schd_last_microsecond = current_microsecond;

	for (idx = 0; idx < MAX_SCHD_ITEMS; ++idx) {
		if (_schd_items[idx].timeout_microsecond > 0
				&& current_microsecond > _schd_items[idx].timeout_microsecond) {

			_schd_items[idx].callback(_schd_items[idx].param);

			_schd_items[idx].timeout_microsecond = 0;
		}
	}
}
