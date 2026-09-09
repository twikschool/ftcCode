(function(window, $, settings) {
	window.env = typeof window.env !== 'undefined' ? window.env : {},
	env.settings = settings;
	env.settings['_dict'] = env.settings.hasOwnProperty('_dict') ? env.settings._dict : {};
	var dict = env.settings._dict;

	settings.get = function(name) {
		return typeof dict[name] === 'undefined' ? null : dict[name];
	}

	settings.put = function(name, val) {
		if (typeof name === 'undefined' || typeof val === 'undefined') throw new Error('put doesn\' work with name or val undefined');
		if (settings.get(name) === null) { console.warn(name + " is not a valid setting"); return; }
		dict[name] = typeof val === 'function' ? val() : val;
		return this;
   }

    settings.commit = function(name, val) {
        return $.post(settings._settingsUrl, 'settings=' + window.JSON.stringify(dict));
    }

   env.urls = JSON.parse(settings._urls)})(window, jQuery, 
{
	_dict: window.JSON.parse('{"autoImportEnabled":true,"softWrap":false,"spacesToTab":4,"printMargin":true,"invisibleChars":false,"keybinding":"OnBotJava","autocompleteForceEnabled":false,"settingEpoch":1,"theme":"chrome","whitespace":"space","fontSize":16,"useNewOnBotJavaWorker":true,"autocompleteEnabled":true,"autocompletePackages":["android/util","org/firstinspires/ftc/ftccommon/external","org/firstinspires/ftc/robotcore/external","org/firstinspires/ftc/vision","com/qualcomm/robotcore/eventloop/opmode","com/qualcomm/robotcore/hardware","com/qualcomm/robotcore/robot","com/qualcomm/robotcore/util","com/qualcomm/hardware","java/io","java/lang","java/math","java/text","java/util"],"font":"Source Code Pro","defaultPackage":"org.firstinspires.ftc.teamcode"}'),
	_settingsUrl: '/java/admin/settings',
   _urls: '{"URI_FILE_DOWNLOAD":"/java/file/download","REQUEST_KEY_ID":"id","URI_ADMIN_RESET_ONBOTJAVA":"/java/admin/factory_reset","URI_JS_AUTOCOMPLETE":"/java/js/editor/autocomplete","URI_BUILD_LOG":"/java/build/log","URI_ADMIN_REARM":"/java/admin/rearm","REQUEST_KEY_TEAM_NAME":"teamName","REQUEST_KEY_FILE":"f","REQUEST_KEY_SETUP_HARDWARE":"rcSetupHardware","REQUEST_KEY_OPMODE_ANNOTATIONS":"opModeAnnotations","URI_FILE_GET_TREE":"/java/file/all","WS_BUILD_LAUNCH":"build:launch","URI_JAVA_PREFIX":"/java","URI_FILE_NEW":"/java/file/new","URI_JAVA_README_FILE":"/java/readme.md","REQUEST_KEY_COPY_FROM":"origin","URI_FILE_TEMPLATES":"/java/file/templates","URI_BUILD_WAIT":"/java/build/wait","URI_FILE_DELETE":"/java/file/delete","URI_ADMIN_SETTINGS":"/java/admin/settings","URI_FILE_SAVE":"/java/file/save","REQUEST_KEY_PRESERVE":"preserve","URI_ADMIN_CLEAN":"/java/admin/clean","URI_JAVA_EDITOR":"/java/editor.html","REQUEST_KEY_SAVE":"data","REQUEST_KEY_TEMPLATE":"template","REQUEST_KEY_COPY_TO":"dest","WS_BUILD_STATUS":"build:status","URI_ADMIN_SETTINGS_RESET":"/java/admin/settings/reset","URI_BUILD_STATUS":"/java/build/status","URI_BUILD_LAUNCH":"/java/build/start","URI_FILE_COPY":"/java/file/copy","REQUEST_KEY_DELETE":"delete","REQUEST_KEY_NEW":"new","URI_FILE_GET":"/java/file/get","URI_FILE_UPLOAD":"/java/file/upload","WS_NAMESPACE":"ONBOTJAVA","URI_FILE_TREE":"/java/file/tree","REQUEST_KEY_OPMODE_NAME":"opModeName","URI_JS_SETTINGS":"/java/js/settings.js"}'
});