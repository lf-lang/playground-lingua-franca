def sorted(s):
    stride = len(s) // 4
    return s[0:stride] + s[2*stride:3*stride] + s[stride:2*stride] + s[3*stride:]

def stride_sorted(s, stride):
    segment_length = stride * 4
    return sum(
        [
            sorted(s[i:j]) for i, j in
            zip(range(0, len(s), segment_length), range(segment_length, len(s) + 1, segment_length))
        ], []
    )

def sorted_with_strides(s, strides):
    for stride in strides:
        s = stride_sorted(s, stride)
    return s


def run(n, strides=[2, 4, 1, 2]):
    a = sorted_with_strides(list(range(n**2)), strides)
    print(a)
    print([x % n for x in a])
