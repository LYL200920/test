
webix.protoUI(
{
  name: "number_param", $cssName:"text",

  defaults:
  {
    template:function(obj, common)
    {
      const name = obj.name || obj.id;
      const value = (obj.value || 0);
      const id = "x" + webix.uid ();
      const input_align = obj.inputAlign || "left";

      obj.min = obj.min || "0";
      obj.max = obj.max || "Infinity";
      obj.step = obj.step || "1";

      var html = common._baseInputHTML("input")
		+ " id='" + id + "' name='" + name + "'"
		+ " type='number' class='webix_inp_counter_value' aria-live='assertive'"
		+ " value='"+value+"' style='width:" + common._get_input_width (obj)+"px; text-align:" + input_align + ";'"
		+ " min='" + obj.min + "'"
		+ " max='" + obj.max + "'"
		+ " step='" + obj.step + "'"
		+ "></input>";


//      var html = "<input id='"+id+"' type='number' class='webix_inp_counter_value' aria-live='assertive'"+" value='"+value+"' style='width:" + common._get_input_width (config)+"px;'></input>";
      return common.$renderInput(obj, html, id);
    },
    min:0,
    max:Infinity,
    step:1
  },

  $init:function()
  {
    // need to hook the mouse wheel event in order and it will magically
    // be handled by the input element.
    webix.event (this.$view, (webix.env.isIE8 ? "mousewheel" : "wheel"), (e) => { });

    // this will trigger the "onChange" event.
    webix.event (this.$view, "input", (e) =>
    {
      this.setValue (this.getValueFromText ());
    });

    webix.event (this.$view, "change", (e) =>
    {
      this.setValue (this.getValueFromText ());
    });

    Object.defineProperty (this, "min",
    {
      set:function (value)
      {
	this._settings.min = value;
	this.$view.min = value;
      },

      get:function ()
      {
	return this._settings.min;
      }
    });

    Object.defineProperty (this, "max",
    {
      set:function (value)
      {
	this._settings.max = value;
	this.$view.max = value;
      },

      get:function ()
      {
	return this._settings.max;
      }
    });
  },

  setValue:function(value)
  {
    const old_value = this._settings.value;
    const new_value = Math.max (Math.min (value, this.max), this.min);
    //console.log ("setValue old: " + old_value + " new: " + new_value);

    this.getInputNode (this.node).value = new_value;

    if (old_value != new_value)
    {
      this._settings.value = new_value;
      this.callEvent("onChange", [new_value]);
    }
  },

  getValueFromText:function()
  {
    return webix.ui.button.prototype.getValue.apply(this,arguments)*1;
  },

  getValue:function()
  {
    return this._settings.value;
  },

  getInputNode:function(){ return this._dataobj.getElementsByTagName("input")[0]; },

/*
  getValue:function(obj){ return  webix.ui.button.prototype.getValue.apply(this,arguments)*1; },

	next:function(step){
		step = this._settings.step;
		this.shift(step);
	},
	prev:function(step){
		step = (-1)*this._settings.step;
		this.shift(step);
	},
	shift:function(step){
		var min = this._settings.min;
		var max = this._settings.max;

		var new_value = this.getValue() + step;
		if (new_value >= min && new_value <= max)
			this.setValue(new_value);
	},

  setMin:function(value)
  { 
  }
*/

}, webix.ui.text);

/*

  <td><input class="number_param" type="number" id="set_feed_val" min="0" max="%d"
	 onchange="" oninput="send_params(this)" onwheel="" /></td></tr>



/*



  next:function(step){ step = this._settings.step; this.shift(step); },

  prev:function(step){ step = (-1)*this._settings.step;	this.shift(step); },

  shift:function(step)
  {
    var min = this._settings.min;
    var max = this._settings.max;

    var new_value = this.getValue() + step;
    if (new_value >= min && new_value <= max)
      this.setValue(new_value);
  }
*/

