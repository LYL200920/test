
const utils = {

isxdigit:function (c)
{
  switch (c)
  {
    default:
      return false;

    case '0': case '1': case '2': case '3': case '4': case '5': case '6':
    case '7': case '8': case '9':
    case 'A': case 'B': case 'C': case 'D': case 'E': case 'F':
    case 'a': case 'b': case 'c': case 'd': case 'e': case 'f':
      return true;
  }
},

isxstr:function (val)
{
  val = val + "";
  let r = true;

  for (let i in val)
    if (!utils.isxdigit (val.charAt (i)))
    {
      r = false;
      break;
    }

  return r;
},

chain_method: function (obj, m, new_m)
{
  const prev_m = obj[m];

  const ovr_func = function (args)
  {
    prev_m.apply (this, arguments);
    return new_m.apply (this, arguments);
  }

  if (obj[m] != new_m)
    obj[m] = ovr_func;
},

round_to_decimal_points: function (value, decimals)
{
  return Number(Math.round(value+'e'+decimals)+'e-'+decimals);
}


} // utils
