window.fbAsyncInit = function () {
  FB.init({
    appId: '974300346334290',
    xfbml: true,
    version: 'v12.0',
  })
  FB.AppEvents.logPageView()
}
;(function (d, s, id) {
  var js,
    fjs = d.getElementsByTagName(s)[0]
  if (d.getElementById(id)) {
    return
  }
  js = d.createElement(s)
  js.id = id
  js.src = 'https://connect.facebook.net/en_US/sdk.js'
  js.onload = () => {
    console.log('Facebook SDK loaded successfully.')
  }
  js.onerror = () => {
    console.error('Failed to load Facebook SDK.')
    // Optionally, notify the user or disable Facebook login options
  }
  fjs.parentNode.insertBefore(js, fjs)
})(document, 'script', 'facebook-jssdk')
