// Language switch: the ko/ and en/ builds share page names, so swap the language folder in the URL.
document.addEventListener('DOMContentLoaded', function () {
  var cur = document.documentElement.lang || (document.querySelector('meta[name="doc-lang"]') || {}).content;
  document.querySelectorAll('a.lang-other').forEach(function (a) {
    var other = a.getAttribute('data-other-lang');
    var re = new RegExp('/' + cur + '/');
    var href = window.location.href;
    a.href = re.test(href) ? href.replace(re, '/' + other + '/') : '../' + other + '/index.html';
  });
});
