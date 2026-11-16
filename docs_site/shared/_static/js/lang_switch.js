// 언어 전환: ko/ 와 en/ 빌드는 페이지 이름이 같으므로 URL 의 언어 폴더만 바꾼다.
document.addEventListener('DOMContentLoaded', function () {
  var cur = document.documentElement.lang || (document.querySelector('meta[name="doc-lang"]') || {}).content;
  document.querySelectorAll('a.lang-other').forEach(function (a) {
    var other = a.getAttribute('data-other-lang');
    var re = new RegExp('/' + cur + '/');
    var href = window.location.href;
    a.href = re.test(href) ? href.replace(re, '/' + other + '/') : '../' + other + '/index.html';
  });
});
