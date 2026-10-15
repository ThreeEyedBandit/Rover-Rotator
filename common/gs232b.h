#pragma once

// Yaesu GS-232B (and the GS-232A layout many "G232b" clones still emit).
//
// Commands are ASCII, terminated by CR (0x0D). A CR is returned after every
// command, plus an LF (0x0A) when the command returned data. LF is ignored.
// Command letters may be either case. Angles are degrees, 3 digits, and 0 is
// the most counterclockwise azimuth (Yaesu GS-232B operating manual).
//
// Position replies this decoder accepts, both documented:
//   C  -> +0aaa          or AZ=aaa          or the calibration form +aaa
//   B  -> EL=eee
//   C2 -> +0aaa+0eee     or AZ=aaa EL=eee
// +0aaa / +0aaa+0eee is the fixed layout called out for C and C2. The GS-232B
// command list prints the same readings as AZ=aaa and AZ=aaa EL=eee. Full-scale
// calibration text in that manual also shows +180 / +450 and +0nn+180.
// Azimuth above 450 or elevation above 180 is rejected rather than invented
// into a position.

static const int GS232B_LINE_MAX = 80;
static const int GS232B_AZ_MAX = 450;
static const int GS232B_EL_MAX = 180;

struct Gs232bParser
{
  char buf[GS232B_LINE_MAX];
  int len;
  bool drop;
};

struct Gs232bPosition
{
  bool hasAz;
  bool hasEl;
  int az;
  int el;
};

inline void gs232bParserReset(Gs232bParser *parser)
{
  parser->len = 0;
  parser->drop = false;
  parser->buf[0] = 0;
}

inline bool gs232bIsDigit(char c)
{
  return c >= '0' && c <= '9';
}

inline char gs232bUpper(char c)
{
  if(c >= 'a' && c <= 'z') return (char)(c - 'a' + 'A');
  return c;
}

inline bool gs232bStartsWith(const char *s, const char *token)
{
  while(*token)
  {
    if(gs232bUpper(*s) != gs232bUpper(*token)) return false;
    s++;
    token++;
  }
  return true;
}

// Returns true when a CR completed a non-empty line that was not discarded.
// The line in `out` does not include the CR. LF and other controls do not
// complete a frame. A line that overflows is dropped through the next CR.
inline bool gs232bPushByte(Gs232bParser *parser, int byteValue, char *out, int outCap)
{
  unsigned char c = (unsigned char)byteValue;
  if(c == '\n') return false;
  if(c == '\r')
  {
    bool emit = !parser->drop && parser->len > 0 && outCap > 1;
    if(emit)
    {
      int n = parser->len;
      if(n >= outCap) n = outCap - 1;
      for(int i = 0; i < n; i++) out[i] = parser->buf[i];
      out[n] = 0;
    }
    parser->len = 0;
    parser->drop = false;
    return emit;
  }
  if(c < 32 || c > 126) return false;
  if(parser->drop) return false;
  if(parser->len >= GS232B_LINE_MAX - 1)
  {
    parser->drop = true;
    parser->len = 0;
    return false;
  }
  parser->buf[parser->len++] = (char)c;
  return false;
}

inline bool gs232bParseDigits(const char *s, int *value, int *used)
{
  if(!gs232bIsDigit(s[0])) return false;
  int v = 0;
  int i = 0;
  while(i < 3 && gs232bIsDigit(s[i]))
  {
    v = v * 10 + (s[i] - '0');
    i++;
  }
  if(gs232bIsDigit(s[i])) return false;
  *value = v;
  *used = i;
  return true;
}

// +0aaa (5 chars) or the calibration reply +aaa (4 chars, e.g. +450).
inline bool gs232bParsePlusAngle(const char *s, int *angle, int *used)
{
  if(s[0] != '+') return false;
  if(s[1] == '0' && gs232bIsDigit(s[2]) && gs232bIsDigit(s[3]) && gs232bIsDigit(s[4]) && !gs232bIsDigit(s[5]))
  {
    *angle = (s[2] - '0') * 100 + (s[3] - '0') * 10 + (s[4] - '0');
    *used = 5;
    return true;
  }
  if(gs232bIsDigit(s[1]) && gs232bIsDigit(s[2]) && gs232bIsDigit(s[3]) && !gs232bIsDigit(s[4]))
  {
    *angle = (s[1] - '0') * 100 + (s[2] - '0') * 10 + (s[3] - '0');
    *used = 4;
    return true;
  }
  return false;
}

inline const char *gs232bSkipSpaces(const char *s)
{
  while(*s == ' ') s++;
  return s;
}

inline bool gs232bDecodePosition(const char *line, Gs232bPosition *out)
{
  out->hasAz = false;
  out->hasEl = false;
  out->az = 0;
  out->el = 0;
  if(!line) return false;

  char tmp[GS232B_LINE_MAX];
  int n = 0;
  while(line[n] && n < GS232B_LINE_MAX - 1)
  {
    tmp[n] = line[n];
    n++;
  }
  tmp[n] = 0;

  int start = 0;
  while(tmp[start] == ' ') start++;
  int end = start;
  while(tmp[end]) end++;
  while(end > start && tmp[end - 1] == ' ') end--;
  tmp[end] = 0;
  const char *s = tmp + start;
  if(*s == 0) return false;

  if(*s == '+')
  {
    int az = 0;
    int used = 0;
    if(!gs232bParsePlusAngle(s, &az, &used)) return false;
    s += used;
    if(*s == 0)
    {
      if(az > GS232B_AZ_MAX) return false;
      out->hasAz = true;
      out->az = az;
      return true;
    }
    int el = 0;
    int usedEl = 0;
    if(!gs232bParsePlusAngle(s, &el, &usedEl)) return false;
    s += usedEl;
    if(*s != 0) return false;
    if(az > GS232B_AZ_MAX || el > GS232B_EL_MAX) return false;
    out->hasAz = true;
    out->hasEl = true;
    out->az = az;
    out->el = el;
    return true;
  }

  if(gs232bStartsWith(s, "AZ="))
  {
    s += 3;
    int az = 0;
    int used = 0;
    if(!gs232bParseDigits(s, &az, &used)) return false;
    s += used;
    if(*s == 0)
    {
      if(az > GS232B_AZ_MAX) return false;
      out->hasAz = true;
      out->az = az;
      return true;
    }
    if(*s != ' ') return false;
    s = gs232bSkipSpaces(s);
    if(!gs232bStartsWith(s, "EL=")) return false;
    s += 3;
    int el = 0;
    int usedEl = 0;
    if(!gs232bParseDigits(s, &el, &usedEl)) return false;
    s += usedEl;
    if(*s != 0) return false;
    if(az > GS232B_AZ_MAX || el > GS232B_EL_MAX) return false;
    out->hasAz = true;
    out->hasEl = true;
    out->az = az;
    out->el = el;
    return true;
  }

  if(gs232bStartsWith(s, "EL="))
  {
    s += 3;
    int el = 0;
    int used = 0;
    if(!gs232bParseDigits(s, &el, &used)) return false;
    s += used;
    if(*s != 0) return false;
    if(el > GS232B_EL_MAX) return false;
    out->hasEl = true;
    out->el = el;
    return true;
  }

  return false;
}

// Usual GS-232B command set. M and W require the manual's 3-digit angles.
// Returns the CR-terminated frame the serial link sends.
inline bool gs232bFormatCommand(char *dest, int cap, const char *cmd)
{
  if(!dest || cap < 3 || !cmd || !cmd[0]) return false;

  bool ok = false;
  char a = gs232bUpper(cmd[0]);
  if(cmd[1] == 0)
  {
    ok = (a == 'C' || a == 'B' || a == 'A' || a == 'E' || a == 'S' ||
          a == 'L' || a == 'R' || a == 'U' || a == 'D' || a == 'O' || a == 'F');
  }
  else if(gs232bUpper(cmd[0]) == 'C' && cmd[1] == '2' && cmd[2] == 0) ok = true;
  else if(a == 'O' && cmd[1] == '2' && cmd[2] == 0) ok = true;
  else if(a == 'F' && cmd[1] == '2' && cmd[2] == 0) ok = true;
  else if(a == 'P' && cmd[1] == '3' && cmd[2] == '6' && cmd[3] == 0) ok = true;
  else if(a == 'P' && cmd[1] == '4' && cmd[2] == '5' && cmd[3] == 0) ok = true;
  else if(a == 'X' && cmd[1] >= '1' && cmd[1] <= '4' && cmd[2] == 0) ok = true;
  else if(a == 'M' && gs232bIsDigit(cmd[1]) && gs232bIsDigit(cmd[2]) && gs232bIsDigit(cmd[3]) && cmd[4] == 0) ok = true;
  else if(a == 'W' && gs232bIsDigit(cmd[1]) && gs232bIsDigit(cmd[2]) && gs232bIsDigit(cmd[3]) &&
          cmd[4] == ' ' && gs232bIsDigit(cmd[5]) && gs232bIsDigit(cmd[6]) && gs232bIsDigit(cmd[7]) && cmd[8] == 0)
    ok = true;

  if(!ok) return false;

  int n = 0;
  while(cmd[n]) n++;
  if(n + 2 > cap) return false;
  for(int i = 0; i < n; i++) dest[i] = gs232bUpper(cmd[i]);
  dest[n] = '\r';
  dest[n + 1] = 0;
  return true;
}
