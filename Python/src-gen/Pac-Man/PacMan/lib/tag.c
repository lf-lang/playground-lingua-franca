#include "../include/ctarget/tag.h"

/**
 * @deprecated version of `lf_tag_compare`
 */
int compare_tags(tag_t tag1, tag_t tag2) {
    return lf_tag_compare(tag1, tag2);
}

/**
 * @deprecated version of `lf_tag`
 */
tag_t get_current_tag(void) {
    return lf_tag();
}

/**
 * Return the current microstep.
 * @deprecated
 */
microstep_t get_microstep() {
    return lf_tag().microstep;
}
