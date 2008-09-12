uniform sampler2D tex;

uniform float gain;
uniform float offset;
uniform float gamma;
uniform int display_channel;  // 0 = RGBA, 1 = R, 2 = G, 3 = B, 4 = A

void main() { 
  vec4 g = vec4(gain, gain, gain, 1.0);
  vec4 o = vec4(offset, offset, offset, 1.0);
  vec4 v = vec4(gamma, gamma, gamma, 1.0);

  vec4 src_tex = texture2D(tex,gl_TexCoord[0].st);
  vec4 selected_tex = src_tex;
  if (display_channel == 1) {
    selected_tex = vec4(src_tex[0],src_tex[0],src_tex[0],1.0);
  } else if (display_channel == 2) {
    selected_tex = vec4(src_tex[1],src_tex[1],src_tex[1],1.0);
  } else if (display_channel == 3) {
    selected_tex = vec4(src_tex[2],src_tex[2],src_tex[2],1.0);
  } else if (display_channel == 4) {
    selected_tex = vec4(src_tex[3],src_tex[3],src_tex[3],1.0);
  } 
  gl_FragColor = pow(g*selected_tex + o, v);
}
