# Pull the converter/scaler code out of the driver verbatim for a userspace test.
import re, sys
src = open(sys.argv[1]).read()
lines = src.split("\n")
def block(start_pat, end_pat):
    a = next(i for i,l in enumerate(lines) if re.match(start_pat, l))
    b = next(i for i in range(a+1, len(lines)) if re.match(end_pat, lines[i]))
    return "\n".join(lines[a:b])
protos = "\n".join(l for l in lines if re.match(r"^static\s+void\s+\w+_?Scaler\(.*\);|^static\s+void\s+VideoScaler\(.*\);|^static\s+void\s+All_VideoScaler\(.*\);", l))
# the "no signal" picture: its font, the text renderer and the function
novid = block(r"^/\* 5x7 glyphs for the \"no signal\" picture", r"^static void SetNoVideoMem") + "\n"
novid += block(r"^static void SetNoVideoMem", r"^}") + "\n}"
conv = block(r"^static void FHD_To_800X600_Scaler", r"^static int hws_take_buffers")
conv = conv[:conv.rindex("\n}")+2]  # drop the comment that precedes hws_take_buffers
# conv ends at the first top-level comment after the converters; make sure we got through FillYUU2
assert "FillYUU2" in conv and "SetDeInterlace" in conv, "extraction too short"
cf = block(r"^static int hws_convert_frame", r"^/\*$")
cf = cf[:cf.rindex("\n}")+2]
print(protos); print(novid); print(conv); print(cf)
